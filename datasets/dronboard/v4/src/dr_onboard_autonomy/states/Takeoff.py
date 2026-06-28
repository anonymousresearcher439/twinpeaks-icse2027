import time
from queue import Queue
from typing import Optional

from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
import rospy
from droneresponse_mathtools import Lla
from pymavlink.dialects.v20 import common as mavlink2


from dr_onboard_autonomy.airlease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla, convert_Lla_to_LlaDict, convert_tuple_to_LlaDict
from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.models import FCUCopterMode, FCUCopterMode, TimeStampedLlaPosition, RelativeAltitude, FCUState
from dr_onboard_autonomy.states import AirLeaser, Offboard, ReadPosition, Arm
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
from dr_onboard_autonomy.state_factory import register_state
from dr_onboard_autonomy.drone import ArdupilotCopterMavrosDrone

from .BaseState import BaseState


_TRANSITION_DELAY = 3.0

"""Example JSON:
{
    "name": "Takeoff",
    "class": "Takeoff",
    "args": {
        "altitude": 7,
        "speed": 1.5,
        "altitude_threshold": 1.0

    },
    "transitions": [
        {
            "target": "BriarHover",
            "condition": "succeeded_takeoff"
        },
        {
            "target": "Land",
            "condition": "failed_takeoff"
        }
    ]
}
"""
@register_state(default_transitions={
    "error": "failure",
    "failed_takeoff": "Land",
    "failsafe": "Failsafe",
    "rtl": "Rtl"
})
class TakeoffArdupilot(BaseState):
    """Specialized takeoff state for Ardupilot drones
    
    """
    # Implementation notes:
    # - This class includes some internal classes. This was done to keep related code together.
    #
    # When it comes to the take off procedure, the following steps are taken:
    # 1. Get required data
    # 2. Request air lease
    # 3. Switch to loiter mode
    # 4. Arm
    # 5. send the Takeoff command
    # 6. wait for the drone to reach the desired altitude and start holding
    # 7. Switch to guided mode
    # 8. start the holding trajectory

    class ModeSwitcher(BaseState):
        """This switches the drone to the mode specified

        The 'succeeded' outcome will occur when the drone's mode is the specified mode
        """

        def __init__(self, mode: FCUCopterMode, **kwargs):
            kwargs["outcomes"] = ["succeeded", "error"]
            super().__init__(**kwargs)

            self.mode = mode

            self.message_senders.add(self.reusable_message_senders.find("state"))
            self.handlers.add_handler("state", self.on_state_message)

        def on_entry(self, userdata):
            rospy.loginfo(f"switching to {self.mode} mode")
            outcome = self.drone.set_fcu_mode(self.mode)
            if not outcome:
                return "error"
            # now we wait for the mode to change

        def on_state_message(self, message):
            drone_state: FCUState = message["data"]
            if drone_state.mode == self.mode:
                return "succeeded"
    
    class MavlinkTakeoff(BaseState):

        def __init__(self, takeoff_altitude: float, altitude_threshold: float, hold_time: float, **kwargs):
            """
            Args:
                takeoff_altitude: meters. height in meters above current altitude

                altitude_threshold: meters. how close to the target altitude the
                    drone must the drone fly to consider takeoff successful

                hold_time: seconds. how long the drone must hold at the target altitude
                    before the takeoff is considered successful

            This will send the MAV_CMD_NAV_TAKEOFF command to the drone. It will then wait for the
            drone to reach the desired altitude. If the drone is within the altitude_threshold of the
            desired altitude for hold_time seconds, the takeoff is considered successful.

            Note: if the drone reaches the desired altitude and then descends below or drifts above
            the threshold, then the timer will reset. The drone must be within the threshold for
            hold_time seconds for the takeoff to be considered successful.
            """
            kwargs["outcomes"] = [
                "succeeded_takeoff",
                "error",
            ]
            super().__init__(**kwargs)

            self.takeoff_altitude = takeoff_altitude
            self.altitude_threshold = altitude_threshold
            self.hold_time = hold_time
            self._arrival_time = None

            self.message_senders.add(self.reusable_message_senders.find("relative_altitude"))
            self.handlers.add_handler("relative_altitude", self.on_relative_altitude)

        def on_entry(self, userdata):
            self.send_takeoff_cmd()
        
        def send_takeoff_cmd(self):
            ## Send the Takeoff command
            rospy.loginfo(f"Taking off to {self.takeoff_altitude} meters")
            # We use a COMAND_LONG message to send the takeoff command
            # COMMAND_LONG DOCS:
            # https://mavlink.io/en/messages/common.html#COMMAND_LONG
            msg = mavlink2.MAVLink_command_long_message(
                target_system=0,
                target_component=0,
                command=mavlink2.MAV_CMD_NAV_TAKEOFF, # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
                confirmation=0,
                param1=0, # pitch (not used with drones)
                param2=0, # empty
                param3=0, # empty
                param4=0, # yaw angle - set to NaN to maintain current heading
                param5=0, # latitude (not used)
                param6=0, # longitude (not used)
                param7=self.takeoff_altitude, # takeoff altitude
            )
            drone: ArdupilotCopterMavrosDrone = self.drone
            drone.send_mavlink(msg)
        
        def on_relative_altitude(self, message):
            alt: RelativeAltitude = message.get("data", float("nan"))
            # if we are within 0.25 meters of the takeoff altitude, we have arrived
            if abs(alt - self.takeoff_altitude) < self.altitude_threshold:
                return self._on_arrival()
            else:
                # if we are not within the threshold, reset the arrival time
                self._arrival_time = None
        
        def _on_arrival(self):
            if self._arrival_time is None:
                self._arrival_time = time.time()
            if time.time() - self._arrival_time > self.hold_time:
                return "succeeded_takeoff"


    def __init__(self, altitude:float=None, speed:float=1.5, altitude_threshold=2.1, **kwargs):
        """
        Args:
            altitude: meters. height in meters above current altitude

            speed: meters per second. This argument is ignored for Ardupilot
                drones. It's here for compatibility the PX4 takeoff state.
                Originally this was used to set the speed of the takeoff trajectory.

            altitude_threshold: meters. how close to the target altitude the
                drone must the drone fly to consider takeoff successful. With
                Ardupilot this value is ignored. It's here for compatibility
                with the PX4 takeoff state. 
        """
        kwargs["outcomes"] = [
            "succeeded_takeoff",
            "failed_takeoff",
            "error",
            "failsafe",
            "rtl"
        ]
        # TODO should we check if drone is an instance of ArdupilotCopterMavrosDrone?
        # assert isinstance(kwargs["drone"], ArdupilotCopterMavrosDrone)
        # TODO find out why we need to disable the heartbeat_handler for takeoff
        kwargs.update({
            "heartbeat_handler": False,
            "trajectory_class": None,
        })
        super().__init__(**kwargs)

        self.altitude_threshold = 2.0

        self.takeoff_altitude = altitude
        self.speed = speed

        if 'name' not in kwargs:
            kwargs['name'] = self.name
        self._pass_through_kwargs = kwargs


        loiter_switch_args = kwargs.copy()
        del loiter_switch_args['trajectory_class']
        del loiter_switch_args['heartbeat_handler']
        loiter_switch_args['mode'] = FCUCopterMode.OFFBOARD
        self.mode_switcher = TakeoffArdupilot.ModeSwitcher(**loiter_switch_args)

        self.arm_state = Arm(**kwargs)
    
        self.mavlink_takeoff_args = kwargs.copy()
        del self.mavlink_takeoff_args['trajectory_class']

        self._air_leaser: Optional[AirLeaser] = None
        self._pos_return_channel = Queue()
        self._position_reader = ReadPosition(return_channel=self._pos_return_channel, **kwargs)

        self.rel_alt: RelativeAltitude = None
        self.position = None
        self._start_position = None
        self._final_position = None

        self.debounce_logger = DebounceLogger()
    
    def on_entry(self, userdata):
        # Order of events:
        # 1. Get required data
        # 2. Request air lease
        # 3. Switch to loiter mode
        # 4. Arm
        # 5. send the Takeoff command
        # 6. wait for the drone to reach the desired altitude and start holding
    
        ## Get required data
        # get takeoff altitude
        if self.takeoff_altitude is None:
            self.takeoff_altitude = self.drone.params.takeoff_altitude

        assert self.takeoff_altitude is not None
        assert self.takeoff_altitude >= 2.5

        # get current position and final position
        outcome = self.execute_substate(self._position_reader, userdata)
        rospy.loginfo(f"Takeoff - initial position retrieval outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        current_pos: BriarLla = self._pos_return_channel.get()
        final_pos: Lla = current_pos.ellipsoid.lla.move_ned(0, 0, -1.0 * self.takeoff_altitude)

        ## Request air lease
        waypoint_air_tunnel_func = make_waypoint_multi_air_tunnel_func(
            LlaPosition(
                final_pos.latitude,
                final_pos.longitude,
                final_pos.altitude,
                datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
            )
        )
        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            is_flying=False,
            unwanted_outcomes={'failsafe', 'rtl'},
            **self._pass_through_kwargs
        )
        outcome = self.execute_substate(self._air_leaser, userdata)
        if outcome != "succeeded":
            return outcome
        rospy.loginfo("Takeoff - air lease granted")

        # ## Switch to loiter mode
        outcome = self.execute_substate(self.mode_switcher, userdata)
        rospy.loginfo(f"loiter mode change outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        
        ## Arm
        outcome = self.execute_substate(self.arm_state, userdata)
        rospy.loginfo(f"arm outcome: {outcome}")
        if outcome != "succeeded_armed":
            return outcome
        

        ## send the Takeoff command and wait
        self.mavlink_takeoff_args.update({
            "takeoff_altitude": self.takeoff_altitude,
            "altitude_threshold": self.altitude_threshold,
            "hold_time": 2.0
        })
        self.mavlink_takeoff = TakeoffArdupilot.MavlinkTakeoff(**self.mavlink_takeoff_args)
        outcome = self.execute_substate(self.mavlink_takeoff, userdata)
        rospy.loginfo(f"takeoff outcome: {outcome}")

        return outcome
        


"""Example JSON:
{
    "name": "Takeoff",
    "class": "Takeoff",
    "args": {
        "altitude": 7,
        "speed": 1.5,
        "altitude_threshold": 1.0

    },
    "transitions": [
        {
            "target": "BriarHover",
            "condition": "succeeded_takeoff"
        },
        {
            "target": "Land",
            "condition": "failed_takeoff"
        }
    ]
}
"""
@register_state(default_transitions={
    "error": "failure",
    "failed_takeoff": "Land",
    "failsafe": "Failsafe",
    "rtl": "Rtl"
})
class TakeoffPx4(BaseState):
    """
    Args:
        altitude: height in meters above current altitude
    """
    def __init__(self, altitude:float=None, speed:float=1.5, altitude_threshold=1.0, **kwargs):
        kwargs["outcomes"] = [
            "succeeded_takeoff",
            "failed_takeoff",
            "error",
            "failsafe",
            "rtl"
        ]
        # TODO find out why we need to disable the heartbeat_handler for takeoff
        kwargs.update({
            "heartbeat_handler": False,
            "trajectory_class": WaypointTrajectory,
        })
        super().__init__(**kwargs)
        self.message_senders.add(self.reusable_message_senders.find("relative_altitude"))
        self.handlers.add_handler("relative_altitude", self.store_rel_alt)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.store_position)
        self.handlers.add_handler("position", self.on_position_message)

        self.message_senders.add(RepeatTimer("print_alt", 0.333))
        self.handlers.add_handler("print_alt", self.log_rel_alt)

        self.altitude_threshold = altitude_threshold
        self.takeoff_altitude = altitude
        self.speed = speed

        if 'name' not in kwargs:
            kwargs['name'] = self.name
        self._pass_through_kwargs = kwargs

        self.offboard_switch = Offboard(**kwargs)
        self.arm_state = Arm(**kwargs)

        self._air_leaser: Optional[AirLeaser] = None
        self._pos_return_channel = Queue()
        self._position_reader = ReadPosition(return_channel=self._pos_return_channel, **kwargs)

        self.rel_alt: RelativeAltitude = None
        self.position = None
        self._start_position = None
        self._final_position = None

        self.debounce_logger = DebounceLogger()


    def on_entry(self, userdata):
        # Order of events:
        # 1. Get required data
        # 2. Request air lease
        # 3. Switch to offboard mode
        # 4. Arm
        # 5. Takeoff

        # get takeoff altitude
        if self.takeoff_altitude is None:
            self.takeoff_altitude = self.drone.params.takeoff_altitude

        assert self.takeoff_altitude is not None

        # get current position and final position
        outcome = self.execute_substate(self._position_reader, userdata)
        rospy.loginfo(f"Takeoff - initial position retrieval outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        current_pos: BriarLla = self._pos_return_channel.get()
        final_pos: Lla = current_pos.ellipsoid.lla.move_ned(0, 0, -1.0 * self.takeoff_altitude)

        # request air lease
        waypoint_air_tunnel_func = make_waypoint_multi_air_tunnel_func(
            LlaPosition(
                final_pos.latitude,
                final_pos.longitude,
                final_pos.altitude,
                datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
            )
        )
        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            is_flying=False,
            unwanted_outcomes={'failsafe', 'rtl'},
            **self._pass_through_kwargs
        ) 
        outcome = self.execute_substate(self._air_leaser, userdata)
        if outcome != "succeeded":
            return outcome
        rospy.loginfo("Takeoff - air lease granted")

        # switch to offboard mode
        outcome = self.execute_substate(self.offboard_switch, userdata)
        rospy.loginfo(f"offboard mode change outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        
        # arm
        outcome = self.execute_substate(self.arm_state, userdata)
        rospy.loginfo(f"arm outcome: {outcome}")
        if outcome != "succeeded_armed":
            return outcome

        rospy.loginfo(f"Taking off to {self.takeoff_altitude} meters")
        self.trajectory.start()
    
    def store_rel_alt(self, message):
        self.rel_alt = message["data"]

    def store_position(self, message):
        pos: TimeStampedLlaPosition  = message["data"]
        pos_tup = pos.latitude, pos.longitude, pos.altitude
        self.position = BriarLla(convert_tuple_to_LlaDict(pos_tup), is_amsl=False)
        if self._start_position is None:
            self._start_position = self.position
            final_pos = self._start_position.ellipsoid.lla.move_ned(
                0,
                0,
                -1.0 * self.takeoff_altitude
            )
            final_pos = convert_Lla_to_LlaDict(final_pos)
            self._final_position = BriarLla(final_pos, is_amsl=False)
            self.trajectory.fly_to_waypoint(self._final_position.llaPosition, self.speed)


    def log_rel_alt(self, _):
        if self.rel_alt is not None:
            self.debounce_logger.info(
                f"Takeoff state: relative altitude: {round(self.rel_alt, 3)} meters. Takeoff ALT = {self.takeoff_altitude} meters.", 1.0, 1
            )
        if self.position is not None:
            alt_amsl = self.position.amsl.lla.altitude
            self.debounce_logger.info(f"Takeoff State: altitude AMSL: {round(alt_amsl, 3)}", 1.0, 2)


    def on_position_message(self, message):
        pos: TimeStampedLlaPosition = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        
        distance = current_pos.distance(self._final_position.ellipsoid.lla)

        self.debounce_logger.info(f"Takeoff State: Current Position: {pos.to_amsl()}", 1.0, 3)
        self.debounce_logger.info(f"Takeoff State: distance to target: {round(distance, 3)} meters", 1.0, 4)
        self.debounce_logger.info(f"Takeoff State: trajectory.is_done() == {self.trajectory.is_done()}", 1.0, 5)

        if distance < self.altitude_threshold and self.trajectory.is_done():
            return "succeeded_takeoff"

@register_state(default_transitions={
    "error": "failure",
    "failed_takeoff": "Land",
    "failsafe": "Failsafe",
    "rtl": "Rtl"
})
class Takeoff(TakeoffPx4):
    pass
