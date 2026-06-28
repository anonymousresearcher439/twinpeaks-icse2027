import math
from math import atan2, radians
from typing import Optional
from droneresponse_mathtools import Lla
from tf.transformations import quaternion_about_axis

import numpy as np

from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, check_angle_with_threshold, distance_is_less_than
from dr_onboard_autonomy.models import TimeStampedLlaPosition
from dr_onboard_autonomy.states import AirLeaser
from dr_onboard_autonomy.states.components import Gimbal
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
from dr_onboard_autonomy.gimbal import GremsyGimbalCalculator

from .BaseState import BaseState

# TODO find a better place to store drone specific constants
DRONE_LEFT_AXIS = GremsyGimbalCalculator.LEFT_AXIS

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


"""Example JSON:

{
    "name": "BriarWaypoint",
    "class": "BriarWaypoint",
    "args":{
        "waypoint": {
            "latitude": 41.60654653889745,
            "longitude": -86.35575685387629,
            "relative_altitude": 22
        },
        "stare_pitch": 45.0,
        "speed": 7.0
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
    "BriarWaypoint",
    "BriarWaypoint2",
    "BriarWaypoint3",
    default_transitions=STANDARD_TRANSITIONS_FLYING,
)
class BriarWaypoint(BaseState):
    def __init__(self,
            waypoint:LlaDict=None,
            stare_position:LlaDict=None,
            stare_pitch:float=None,
            speed:float=2.0,
            **kwargs
        ):
        """
        Fly to the specified waypoint taking the most direct path.

        Args:
            waypoint: the final destination. Alt is AMSL
            stare_position: where the camera should look. Alt is AMSL
            stare_pitch: what pitch should we set the gimbal to? This is in degrees around the
                drone's LEFT axis. Therefore, a positive number like 45° causes the camera to
                look down 45°. If you set this and stare_position then stare_position takes priority.
            speed: how fast to fly in meters per second
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_waypoints"], STANDARD_TRANSITIONS_FLYING.keys(), kwargs.get("unwanted_outcomes", set()))
        

        self.waypoint = BriarLla(waypoint)
        self.speed=float(speed)
        self._pass_through_kwargs = kwargs
        self.trajectory: WaypointTrajectory

        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        self.stare_position = None
        self.stare_pitch = None

        if stare_position is not None or stare_pitch is not None:
            self.gimbal_driver = Gimbal(self)

        if stare_pitch is not None:
            self.stare_pitch = radians(stare_pitch)

        if stare_position is not None:
            self.stare_position: BriarLla = BriarLla(stare_position)
            self.stare_pitch = None

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self._air_leaser: Optional[AirLeaser] = None
        self._current_position = None

        self.debounce_logger = DebounceLogger()

        self.mission_builder: 'MissionBuilder' = kwargs["mission_builder"]


    def on_entry(self, userdata):
        waypoint_air_tunnel_func = make_waypoint_multi_air_tunnel_func(
            end_pos=self.waypoint.ellipsoid.lla
        )
        air_lease_args = self.mission_builder.build_kwargs({
            'tunnel_func': waypoint_air_tunnel_func,
            'name': self.name,
            'outcomes': self._pass_through_kwargs['outcomes'],
        })
        if self.task_id is not None:
            air_lease_args['task_id'] = self.task_id
        self._air_leaser = AirLeaser(**air_lease_args)
        outcome = self.execute_substate(self._air_leaser, userdata)
        if outcome != "succeeded":
            return outcome

        self.trajectory.start()
        self.trajectory.fly_to_waypoint(self.waypoint.amsl.lla, self.speed)

        if self.stare_position or self.stare_pitch:
            self.gimbal_driver.start()

        if self.stare_position:
            self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
        elif self.stare_pitch:
            self.gimbal_driver.fixed_direction = quaternion_about_axis(
                self.stare_pitch,
                DRONE_LEFT_AXIS
            )


    def on_position_message(self, message):
        pos: TimeStampedLlaPosition = message["data"]
        self._current_position = Lla(pos.latitude, pos.longitude, pos.altitude)
        distance = self._current_position.distance(self.waypoint.ellipsoid.lla)
        self.debounce_logger.info(
            f"BriarWaypoint: Current Position: {pos.to_amsl()}",
            1.0,
            1
        )
        self.debounce_logger.info(f"BriarWaypoint: Distance to waypoint: {distance}", 1.0, 2)

        if distance < 1.5 and self.trajectory.is_done():
            return "succeeded_waypoints"

        if not self.stare_position:
            north, east, down = self._current_position.distance_ned(self.waypoint.ellipsoid.lla)
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


    def on_exit(self, outcome, userdata):
        self._air_leaser.communicate_route_complete(
            current_position=self._current_position
        )



from dr_onboard_autonomy.mission_helper import MissionBuilder
