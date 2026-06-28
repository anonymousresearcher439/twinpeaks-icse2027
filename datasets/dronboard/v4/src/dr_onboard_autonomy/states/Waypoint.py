import enum
import math
from math import atan2, radians
from typing import Optional
import time

from droneresponse_mathtools import Lla
from tf.transformations import quaternion_about_axis
import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory
# NOTE: using the pytransitions library
from transitions import Machine, State


from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition

from dr_onboard_autonomy.airlease import (
    AirLeaseCallback,
    AirLeaseOutcome,
    AirspaceSegment,
    AirspaceVolume,
    AirspaceVolumeName,
    NO_OP,
)

from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, check_angle_with_threshold, distance_is_less_than
from dr_onboard_autonomy.models import TimeStampedLlaPosition, CopterParameters
from dr_onboard_autonomy.states.components import AirLease, Setpoint, UpdateStarePosition
from dr_onboard_autonomy.states.components import Gimbal
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
from dr_onboard_autonomy.gimbal import GremsyGimbalCalculator

from .BaseState import BaseState

# TODO find a better place to store drone specific constants
DRONE_LEFT_AXIS = GremsyGimbalCalculator.LEFT_AXIS

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


"""Example JSON:

{
    "name": "Waypoint",
    "class": "Waypoint",
    "args":{
        "waypoint": {
            "latitude": 41.60654653889745,
            "longitude": -86.35575685387629,
            "relative_altitude": 22
        },
        "speed": 7.0
        "stare_pitch": 45.0,
        "incremental_segment_time": 5.0,
        "incremental_handoff_percent": 40.0
    },
    "transitions": [
        {
            "target": "FlyWaypoints",
            "condition": "succeeded_waypoints"
        }
    ]
}

"""


@register_state(
    "Waypoint",
    default_transitions=STANDARD_TRANSITIONS_FLYING,
)
class Waypoint(BaseState):
    """Fly to a waypoint taking the most direct path using incremental air leasing

    This state allows a drone to move toward a waypoint while requesting
    airspace in small incremental segments rather than reserving the entire
    flight path upfront. This approach prevents excessive airspace allocation
    and helps drones share the airspace more efficiently.

    Pre-condition:
    the drone is hovering at its starting position.

    Behavior:
    fly in a straight line toward the waypoint using incremental air leasing.
    The drone requests a volume of airspace that covers only the next few
    seconds of flight. As it moves forward, it requests a replacement airspace
    segment that covers the next few seconds of flight. This process continues
    until the drone reaches its waypoint. 
    
    Post-condition:
    the drone is hovering at the waypoint.

    How it works:
    We do a calculation with the `incremental_segment_time` argument to find the
    boundaries of each segment.

    The `incremental_segment_time` value is in seconds.

    We create a `planning trajectory` that begins with the drones current state
    (its position, velocity, and acceleration) and ends with the destination. We
    then look ahead in the trajectory to calculate the far end of the airspace
    segment. The `incremental_segment_time` value is used to determine how far
    into the future we should look. The airspace segment extends from the
    drone's current position to the future position identified by the planning
    trajectory.

    Once we receive an incremental air lease, we update our "active trajectory"
    so that it takes us to the far end of the segment. Note: the active
    trajectory is the trajectory that the drone is currently following.

    Immediately after updating our active trajectory we calculate the
    `handoff_position`. We use the `incremental_handoff_percent` argument to
    determine how far ahead we should move before requesting a new air lease.
    The `incremental_handoff_percent` value is a percentage. We look at the
    duration of our active trajectory, and we multiply that by
    `incremental_handoff_percent` to find the `handoff_time`. For example if the
    active trajectory will take 5 seconds and the incremental_handoff_percent is
    40% then the handoff time will be 40% * 5 seconds = 2 seconds. We then look
    ahead in the active trajectory by the `handoff_time` to find the
    `handoff_position`.

    The `handoff_position` is a point along the path. When the drone moves
    beyond this point it's time to request a new air lease.

    When the final air lease is approved the drone does not calculate a
    `handoff_position`. Instead it simply flies to the waypoint.
    """
    def __init__(self,
            waypoint:LlaDict=None,
            stare_position:LlaDict=None,
            stare_pitch:float=None,
            speed:float=2.0,
            incremental_segment_time:float=5.0, # in seconds
            incremental_handoff_percent:float=40, # in percent
            **kwargs
        ):
        """
        Initialize the Waypoint state

        Args:

            waypoint (LlaDict): The destination in latitude, longitude, and
            altitude (AMSL). This is where you want the drone to fly.
            
            stare_position (LlaDict, optional): The position where the camera
            should look. (AMSL).  If both `stare_pitch` and `stare_position` are
            provided, `stare_position` takes priority.
            
            stare_pitch (float, optional): The pitch angle for the gimbal in
            degrees around the drone's LEFT axis. A positive value (e.g., 45°)
            makes the camera look downward by that angle. If both `stare_pitch`
            and `stare_position` are provided, `stare_position` takes priority.

            speed (float): The maximum flight speed in meters per second.
            Defaults to 2.0 m/s. This is the speed limit for the trajectory.

            incremental_segment_time (float, optional): The number of seconds to
            look ahead when planning each incremental airspace segment. This
            value determines how far into the planning trajectory the drone will
            request its next airspace volume. For example, if set to 5.0, the
            drone will request a segment that covers the next 5 seconds of
            flight. Defaults to 5.0.

            incremental_handoff_percent (float, optional): The percentage of the
            current segment's duration after which the drone should request the
            next segment. For example, if the current trajectory will take 5
            seconds and this value is 40.0, the drone will begin asking for the
            next segment after 2 seconds (40% of 5 seconds). Defaults to 40.0.
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_waypoints"], STANDARD_TRANSITIONS_FLYING.keys(), kwargs.get("unwanted_outcomes", set()))

        self.waypoint = BriarLla(waypoint)
        self.speed=float(speed)
        self.incremental_segment_time = float(incremental_segment_time)
        self.incremental_handoff_percent = float(incremental_handoff_percent) / 100.0
        self.trajectory: WaypointTrajectory
        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        self.airlease = AirLease(self, False)
        self.debounce_logger = DebounceLogger()

        self.stare_position = None
        self.stare_pitch = None
        if stare_position is not None or stare_pitch is not None:
            self.gimbal_driver = Gimbal(self)
            self.stare_position_updater = UpdateStarePosition(self, self.gimbal_driver)

        if stare_pitch is not None:
            self.stare_pitch = radians(stare_pitch)

        if stare_position is not None:
            self.stare_position: BriarLla = BriarLla(stare_position)
            self.stare_pitch = None

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self.exit_on_deadlock = 'deadlock' in kwargs["outcomes"]
        self.is_deadlocked = False

        self.waypoint_state_machine = IncrementalWaypoint(
            self,
            self.waypoint.llaPosition,
            self.speed,
            self.incremental_segment_time,
            self.incremental_handoff_percent
        )

    def on_entry(self, userdata):
        self.trajectory.start()

        if self.stare_position or self.stare_pitch:
            self.gimbal_driver.start()
        if self.stare_position:
            self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
        elif self.stare_pitch:
            quat = quaternion_about_axis(self.stare_pitch, DRONE_LEFT_AXIS)
            self.gimbal_driver.fixed_direction = quat
    
    def execute2(self, userdata):
        if self.is_deadlocked and self.exit_on_deadlock:
            return "deadlock"
        self.waypoint_state_machine.run()

    def on_position_message(self, message):
        pos: TimeStampedLlaPosition = message["data"]
        
        distance = self._distance_to_waypoint(pos)
        self.debounce_logger.info(f"Waypoint: Distance to waypoint: {distance}", 1.0, 2)

        if distance < 1.5 and self.trajectory.is_done():
            return "succeeded_waypoints"

        if not self.stare_position:
            current_position = _llaPos_to_Lla(pos)
            north, east, down = current_position.distance_ned(self.waypoint.ellipsoid.lla)
            is_far_enough = not distance_is_less_than(north, east, down, 1.0)
            is_not_vertical = not check_angle_with_threshold((north, east, down), threshold=2)
            if is_far_enough and is_not_vertical:
                # the drone is headed nearly up or down
                # therefore we won't set a yaw
                # self.debounce_logger.info("Drone is headed nearly up or down.", 0.5)
                self.trajectory.yaw = atan2(north, east)

        if distance < 1.5:
            self.debounce_logger.info("Waypoint reached, but trajectory is not done yet.", 0.5)
        if self.trajectory.is_done():
            self.debounce_logger.info("Trajectory is done, but waypoint is too far away.", 0.5)
    
    def _distance_to_waypoint(self, pos: TimeStampedLlaPosition) -> float:
        current_position = Lla(pos.latitude, pos.longitude, pos.altitude)
        return current_position.distance(self.waypoint.ellipsoid.lla)
        
from dr_onboard_autonomy.mission_helper import MissionBuilder


def _llaPos_to_Lla(lla_pos: LlaPosition) -> Lla:
    return Lla(lla_pos.latitude, lla_pos.longitude, lla_pos.altitude)

def _lla_to_LlaPos(lla: Lla) -> LlaPosition:
    """internal converter. Assumes the altitude is in ellipsoid height.
    """
    return LlaPosition(lla.latitude, lla.longitude, lla.altitude, datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84)

def _is_past_handoff(start_wgs84: Lla, drone_wgs84: Lla, handoff_wgs84: Lla, waypoint_wgs84: Lla) -> bool:
    
    """
    This function returns true if the drone has made it far enough along the path that it's time to ask for a new segment of airspace

    We determine when the drone has crossed beyond the handoff point using a
    math trick.

    The handoff point is used to anchor the **handoff plane**. The handoff plane
    is a 2D plane in 3D space. The plane is perpendicular to the trajectory. We
    use the drone's direction of motion to define the plane's normal vector.
    When the drone crosses the handoff plane it's time to request the next
    incremental air lease. 

    Implementation Notes:
    Let X be the vector that points from the start to destination:
    X = destination.ecef - start.ecef

    And let U be the vector pointing from the next_handoff_position to the drone's current position:
    U = drone.ecef - handoff_position.ecef

    We take the dot product
    X dot U

    If X dot U ≥ 0, the drone has moved beyond the handoff position.
    If X dot U < 0, the drone hasn't reached the start of the segment.
    """

    start_to_dest = waypoint_wgs84.to_pvector().xyz - start_wgs84.to_pvector().xyz
    handoff_to_drone = drone_wgs84.to_pvector().xyz - handoff_wgs84.to_pvector().xyz
    return np.dot(start_to_dest, handoff_to_drone) >= 0

def _is_forward_progress(start_wgs84: Lla, waypoint_wgs84: Lla, drone_wgs84: Lla, endpoint_wgs84: Lla) -> bool:
    """
    This function returns true if the drone will make forward progress by flying to the endpoint

    NOTE: this function assumes the endpoint is a point along the trajectory from the start to the waypoint
    """
    start_to_dest = waypoint_wgs84.to_pvector().xyz - start_wgs84.to_pvector().xyz
    drone_to_endpoint = endpoint_wgs84.to_pvector().xyz - drone_wgs84.to_pvector().xyz
    # we will use the dot product to see if forward progress is possible by moving to the endpoint
    # if the dot product > 0 then forward progress is possible
    # if the dot product < 0 then forward progress is not possible
    # if the dot product == 0 then the drone is already at the endpoint (forward progress is not possible)
    return np.dot(start_to_dest, drone_to_endpoint) > 0


class WaypointState(enum.Enum):
    READY_CHECK = enum.auto()
    REQUESTING = enum.auto()
    FLYING = enum.auto()

internal_states = [
    State(
        name=WaypointState.READY_CHECK,
        on_enter=["on_enter_ready_check"],
        on_exit=["on_exit_ready_check"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=WaypointState.REQUESTING,
        on_enter=["on_enter_requesting"],
        on_exit=["on_exit_requesting"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=WaypointState.FLYING,
        on_enter=["on_enter_flying"],
        on_exit=["on_exit_flying"],
        ignore_invalid_triggers=False,
        final=False,
    ),
]

transitions = [
    {
        "trigger": "ready", 
        "source": WaypointState.READY_CHECK,
        "dest": WaypointState.REQUESTING,
    },
    {
        "trigger": "flying",
        "source": WaypointState.REQUESTING,
        "dest": WaypointState.FLYING,
    },
    {
        "trigger": "handoff_reached",
        "source": WaypointState.FLYING,
        "dest": WaypointState.REQUESTING,
    },
    {
        "trigger": "retry_request",
        "source": WaypointState.REQUESTING,
        "dest": "=", 
    },
    {
        # there is an edge case where we can get a better air lease while in the FLYING state....
        "trigger": "update_flight_plan", 
        "source": WaypointState.FLYING,
        "dest": "=", # special value for a "Reflexive transition"
    },
    {
        "trigger": "run",
        "source": WaypointState.READY_CHECK,
        "dest": None,
        "before": "on_ready_check",
    },
    {
        "trigger": "run",
        "source": WaypointState.REQUESTING,
        "dest": None,
        "before": "on_requesting",
    },
    {
        "trigger": "run",
        "source": WaypointState.FLYING,
        "dest": None,
        "before": "on_flying",
    },
]


class DurationTimer:
    def __init__(self):
        self.start_time: int = -1
        self.duration: int = -1

    def reset(self):
        """reset the timer
        """
        self.start_time = -1
        self.duration = -1

    def start(self, duration: float):
        """start the timer with a duration in seconds
        """
        self.start_time = time.monotonic_ns()
        self.duration = round(duration * 1e9)
    
    def is_done(self) -> bool:
        """return true if the timer is done
        """
        if self.start_time < 0:
            # we haven't started yet
            return False 
        delta = self._delta_ns()
        return delta >= self.duration

    def _delta_ns(self) -> int:
        """return how much time has passed since you calld start() in seconds
        """
        if self.start_time < 0:
            # we haven't started yet
            return 0
        return time.monotonic_ns() - self.start_time

    def delta(self) -> float:
        """return how much time has passed since you calld start() in seconds
        """
        result = self._delta_ns()
        return result / 1e9


class HandoffStatus(enum.Enum):
    UNKNOWN = enum.auto()
    CALCULATING = enum.auto()
    READY = enum.auto()
    NOT_REQUIRED = enum.auto()


class IncrementalWaypoint:
    """Component to implement the incremental air leasing for the Waypoint state.
    """
    AIR_LEASE_RETRY_PERIOD = 1.0 # seconds

    def __init__(
            self,
            state: Waypoint,
            waypoint: LlaPosition,
            speed: float,
            incremental_segment_time: float,
            incremental_handoff_percent: float
        ):
        self.dr_state = state
        self.trajectory: WaypointTrajectory = state.trajectory
        self.airlease: AirLease = state.airlease
        self.drone = state.drone
        self.start_wgs84: Optional[Lla] = None
        self.waypoint_wgs84 = _llaPos_to_Lla(waypoint.to_wgs84_ellipsoid())
        self.speed = speed
        self.incremental_segment_time = incremental_segment_time
        self.incremental_handoff_percent = incremental_handoff_percent

        self.handoff_status = HandoffStatus.UNKNOWN
        self.handoff_wgs84: Optional[Lla] = None

        self.debounce_logger = DebounceLogger()
        self.timer = DurationTimer()
        self.machine = Machine(
            model=self,
            states=internal_states,
            transitions=transitions,
            initial=WaypointState.READY_CHECK,
            # auto_transitions=False, # for debugging purposes
        )
    
    @property
    def drone_wgs84(self) -> Lla:
        return _llaPos_to_Lla(self.drone.data.location.get_position().to_wgs84_ellipsoid())
    
    def on_enter_ready_check(self):
        self.on_ready_check()
    
    def on_exit_ready_check(self):
        if self.start_wgs84 is None:
            # we need to set the start position
            self.start_wgs84 = _llaPos_to_Lla(self.drone.data.location.get_position())

    def on_ready_check(self):
        # we need to remain in this state until our trajectory implementation is ready
        # the trajectory is "ready" in case:
        # 1. the trajectory is in the MOVING state. If we're moving then we're ready to go
        # 2. the trajectory is in the HOLDING state. Trajectory instances go to this state when they're done STARTING or If we're holding this means either a trajectory to a target location has finished
        ready_states = [
            # when the trajectory object is in one of these states
            # we're ready to start the moving
            WaypointTrajectory.State.MOVING,
            WaypointTrajectory.State.HOLDING,
        ]
        if self.trajectory.state() in ready_states:
            self.ready()
    
    def on_enter_requesting(self):
        AIR_LEASE_RETRY_PERIOD = IncrementalWaypoint.AIR_LEASE_RETRY_PERIOD
        self.debounce_logger.info("Waypoint: Requesting incremental airspace", AIR_LEASE_RETRY_PERIOD)
        airspace = self.find_segment()
        name = self.airlease.request_airspace(airspace, self.on_air_lease_outcome)
        self.debounce_logger.info(f"Waypoint: Asked for airspace: {name}", AIR_LEASE_RETRY_PERIOD, key=1745332417545252680)
        self.timer.start(1.0)

    def on_air_lease_outcome(self, name: AirspaceVolumeName, volume: AirspaceVolume, outcome: AirLeaseOutcome):
        if outcome.deadlock:
            self.dr_state.is_deadlocked = True
        if not outcome.approved:
            return
        if self.state == WaypointState.REQUESTING:
            # check if we can make forward progress with this air lease
            start_wgs84 = self.start_wgs84
            waypoint_wgs84 = self.waypoint_wgs84
            drone_wgs84 = self.drone_wgs84
            endpoint_wgs84 = _llaPos_to_Lla(volume[-1].end.to_wgs84_ellipsoid())

            assert start_wgs84 is not None, "start_wgs84 should not be None"
            assert waypoint_wgs84 is not None, "waypoint_wgs84 should not be None"
            assert drone_wgs84 is not None, "drone_wgs84 should not be None"
            assert endpoint_wgs84 is not None, "endpoint_wgs84 should not be None"

            # check if we can make forward progress
            if not _is_forward_progress(start_wgs84, waypoint_wgs84, drone_wgs84, endpoint_wgs84):
                msg = f"Waypoint: We cannot make forward progress with airlease: {name}, {volume}"
                self.debounce_logger.warn(msg, 0.0001, key=1745331269108499135)
                return
            
            self.trajectory.fly_to_waypoint(volume[-1].end, self.speed)
            self.flying()

        elif self.state == WaypointState.FLYING:
            # this is an edge case. Suppose we're in the REQUESTING state 
            # 1. the drone asks for a new air lease (so it can make forward progress... suppose this air lease ends 30m away from the starting position)
            # 2. the drones waits and waits but no response... 
            # 3. the drone asks for a new air lease for even more forward progress (suppose this air lease ends 40m away from the starting position)
            # 4. we find out that the first air lease was approved. In this case we will enter the FLYING state. We will start flying to the endpoint that's 30m along the path. NOTE: the air leasing component is still trying to get the second air lease!
            # 5. the 2nd air lease gets approved. This is the edge case we're trying to handle here. 

            # In this case we will:
            # 1. check if the new air lease will help us make forward progress
            # 2. check that this new air lease is actually better than the one we already have...
            # if either condition is not met we will log warning.

            # TODO Should we introduce special handling in case our new air lease is worse???
            # for example, should we stop, ask for a hover volume, and then resume?

            start_wgs84 = self.start_wgs84
            waypoint_wgs84 = self.waypoint_wgs84
            drone_wgs84 = self.drone_wgs84
            endpoint_wgs84 = _llaPos_to_Lla(volume[-1].end.to_wgs84_ellipsoid())

            assert start_wgs84 is not None, "start_wgs84 should not be None"
            assert waypoint_wgs84 is not None, "waypoint_wgs84 should not be None"
            assert drone_wgs84 is not None, "drone_wgs84 should not be None"
            assert endpoint_wgs84 is not None, "endpoint_wgs84 should not be None"

            # check if we can make forward progress
            is_forward_from_drone = _is_forward_progress(start_wgs84, waypoint_wgs84, drone_wgs84, endpoint_wgs84)
            if not is_forward_from_drone:
                msg = f"Waypoint: We cannot make forward progress with airlease: {name}, {volume}"
                self.debounce_logger.warn(msg, 0.0001, key=1745331269108499135)
            
            # check if the new endpoint is better than the old one
            old_endpoint = self.trajectory.target_position()
            if old_endpoint is None:
                # we don't have an old endpoint
                # this shouldn't happen but just in case
                msg = f"Waypoint: We don't have a trajectory target but we're in the FLYING state??? THIS SHOULD NOT HAPPEN"
                self.debounce_logger.warn(msg, 0.0001, key=1745331269108499136)
            else:
                old_endpoint = _llaPos_to_Lla(old_endpoint)
                is_forward_from_old_target = _is_forward_progress(start_wgs84, waypoint_wgs84, old_endpoint, endpoint_wgs84)
                if not is_forward_from_old_target:
                    self.debounce_logger.warn("Waypoint: The new air lease is not better than the old one", 0.0001, key=1745331269108499137)

            if old_endpoint and endpoint_wgs84:
                # log the details of the new air lease
                dist_old = old_endpoint.distance(waypoint_wgs84)
                dist_new = endpoint_wgs84.distance(waypoint_wgs84)
                dist_old, dist_new = round(dist_old, 2), round(dist_new, 2)

                msg = f"Waypoint: new air lease. The new airspace get's us to {dist_new}m from the waypoint, the old one gets us to {dist_old}m from the waypoint"

                self.debounce_logger.warn(msg, 0.0001, key=1745331269108499138)
            
            self.trajectory.fly_to_waypoint(volume[-1].end, self.speed)
            self.update_flight_plan()

    def on_requesting(self):
        if self.timer.is_done():
            self.retry_request()
    
    def on_exit_requesting(self):
        self.timer.reset()
    
    def trajectory_calculator(self):
        drone_pos = self.drone.data.location.get_position()
        assert drone_pos is not None, "Drone position is None" # This should never happen
        
        drone_state = self.trajectory.kinematic_state(0.0) 
        origin_wgs84 = _llaPos_to_Lla(drone_state.origin.to_wgs84_ellipsoid()) # ellipsoid lla
        waypoint_ned = origin_wgs84.distance_ned(self.waypoint_wgs84)

        inp: InputParameter = InputParameter(3)
        
        # starting state
        inp.current_position = drone_state.position
        inp.current_velocity = drone_state.velocity
        inp.current_acceleration = drone_state.acceleration

        # target state
        inp.target_position = [float(x) for x in waypoint_ned]
        inp.target_velocity = [0.0, 0.0, 0.0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        # constraints
        inp.max_velocity = [self.speed, self.speed, self.speed]
        constraints = self.drone.params
        max_horizontal_acceleration = constraints.horizontal_acceleration_limit
        max_down_acceleration = constraints.downward_acceleration_limit
        max_up_acceleration = constraints.upward_acceleration_limit
        inp.max_acceleration = [
            max_horizontal_acceleration,
            max_horizontal_acceleration,
            max_down_acceleration
        ]
        inp.min_acceleration = [
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_up_acceleration
        ]
        max_jerk = constraints.jerk_limit
        inp.max_jerk = [
            max_jerk,
            max_jerk,
            max_jerk
        ]

        otg = Ruckig(3)
        trajectory_calculator = Trajectory(3)

        calc_result = otg.calculate(inp, trajectory_calculator)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        return trajectory_calculator, origin_wgs84, drone_pos

    def find_segment(self):
        trajectory_calculator, origin_wgs84, drone_pos = self.trajectory_calculator()
        # Now that we have trajectory_calculator we need to look ahead in time
        # by the incremental_segment_time        
        t = self.incremental_segment_time
        displacement, _, _ = trajectory_calculator.at_time(t)
        endpoint_wgs84 = origin_wgs84.move_ned(*displacement)

        # clamp the endpoint to the waypoint in case it's less than 1m away
        # TODO should we do this?
        if self.waypoint_wgs84.distance(endpoint_wgs84) < 1.0:
            endpoint_wgs84 = self.waypoint_wgs84
        endpoint = _lla_to_LlaPos(endpoint_wgs84)

        # now we can define our airspace volume
        airspace: AirspaceVolume = [
            AirspaceSegment(
                start=drone_pos,
                end=endpoint,
                radius=5.0
            )
        ]
        return airspace
    
    def on_enter_flying(self):
        self.debounce_logger.info("Waypoint: Flying towards waypoint", 0.5)

        # reset handoff fields
        self.handoff_status = HandoffStatus.UNKNOWN
        self.handoff_wgs84 = None

        # we need to check if we need a handoff position
        # we need a handoff position in case the air leasse endpoint != waypoint
        # we will assume that the air lease endpoint is the waypoint in case the distance < 0.01
        # this is to avoid issues related to numerical precision
        endpoint = self.airlease.airspace[-1].end.to_wgs84_ellipsoid()
        endpoint_wgs84 = _llaPos_to_Lla(endpoint)
        if self.waypoint_wgs84.distance(endpoint_wgs84) < 0.01:
            self.handoff_status = HandoffStatus.NOT_REQUIRED
            self.handoff_wgs84 = None
        else:
            self.handoff_status = HandoffStatus.CALCULATING
        self.on_flying()
    
    def on_exit_flying(self):
        self.handoff_status = HandoffStatus.UNKNOWN
        self.handoff_wgs84 = None
    
    def on_flying(self):
        """call this frequently while we're flying to the far end of a segment
        """
        if self.handoff_status == HandoffStatus.CALCULATING:
            self.calculate_handoff()
        
        elif self.handoff_status == HandoffStatus.READY:
            # we need to check if we're past the handoff position
            start_wgs84 = self.start_wgs84
            drone_wgs84 = self.drone_wgs84
            handoff_wgs84 = self.handoff_wgs84
            waypoint_wgs84 = self.waypoint_wgs84
            assert start_wgs84 is not None, "start_wgs84 should not be None"
            assert drone_wgs84 is not None, "drone_wgs84 should not be None"
            assert handoff_wgs84 is not None, "handoff_wgs84 should not be None"
            assert waypoint_wgs84 is not None, "waypoint_wgs84 should not be None"
            is_past = _is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, waypoint_wgs84)
            if is_past:
                self.handoff_reached()
            return
        else:
            # we don't need to do anything here
            # expectation:
            assert self.handoff_status in [HandoffStatus.NOT_REQUIRED, HandoffStatus.UNKNOWN]
            
            return
    
    def calculate_handoff(self):
        # we need to find the handoff position by looking ahead.
        # We need to use our actual trajectory so check that our actual trajectory is in the MOVING state
        if self.trajectory.state() != WaypointTrajectory.State.MOVING:
            # if we're not moving then we can't calculate the handoff position
            # we're not moving in case our trajectory is still initializing
            return
        
        # if we made it this far then we can find the handoff position
        t = self.incremental_segment_time * self.incremental_handoff_percent
        handoff_state = self.trajectory.kinematic_state(t)

        displacement = handoff_state.position
        origin_wgs84 = _llaPos_to_Lla(handoff_state.origin.to_wgs84_ellipsoid())

        self.handoff_wgs84 = origin_wgs84.move_ned(*displacement)
        self.handoff_status = HandoffStatus.READY
    
    



