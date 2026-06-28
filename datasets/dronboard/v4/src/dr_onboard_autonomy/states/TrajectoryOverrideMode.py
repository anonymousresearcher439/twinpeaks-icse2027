import math
from math import atan2, radians
from typing import Optional
from droneresponse_mathtools import Lla
from tf.transformations import quaternion_about_axis

import numpy as np

from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict
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
    "name": "TrajectoryOverrideMode",
    "class": "TrajectoryOverrideMode",
    "args":{
        "stare_position": {
            "latitude": 41.60654653889745,
            "longitude": -86.35575685387629,
            "relative_altitude": 0.0
        },
        "stare_pitch": 45.0,
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

    default_transitions=STANDARD_TRANSITIONS_FLYING,
)
class TrajectoryOverrideMode(BaseState):
    def __init__(self,
            stare_position:LlaDict=None,
            stare_pitch:float=None,
            **kwargs
        ):
        """
        THIS IS A TESTING MODE

        The drone will accept trajectory commands and instantly override the current trajectory.


        Args:
            stare_position: where the camera should look. Alt is AMSL. This is optional.
            stare_pitch: what pitch should we set the gimbal to? This is in degrees around the
                drone's LEFT axis. Therefore, a positive number like 45° causes the camera to
                look down 45°. If you set this and stare_position then stare_position takes priority.
                This is optional.
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["done"], STANDARD_TRANSITIONS_FLYING.keys(), kwargs.get("unwanted_outcomes", set()))
        
        self._pass_through_kwargs = kwargs
        self.trajectory: WaypointTrajectory

        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        self.trajectory_commander = TrajectoryCommander(self, **kwargs)
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

        self.debounce_logger = DebounceLogger()

        self.mission_builder: 'MissionBuilder' = kwargs["mission_builder"]


    def on_entry(self, userdata):
        self.trajectory.start()

        if self.stare_position or self.stare_pitch:
            self.gimbal_driver.start()

        if self.stare_position:
            self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
        elif self.stare_pitch:
            pitch_quat = quaternion_about_axis(self.stare_pitch, DRONE_LEFT_AXIS)
            self.gimbal_driver.fixed_direction = pitch_quat

    def on_position_message(self, message):
        pos: TimeStampedLlaPosition = message["data"]
        self._current_position = Lla(pos.latitude, pos.longitude, pos.altitude)
        self.debounce_logger.info(
            f"TrajectoryOverrrideMode: Current Position: {pos.to_amsl()}",
            1.0,
            1
        )

        # if not self.stare_position:
        #     north, east, down = self._current_position.distance_ned(self.waypoint.ellipsoid.lla)
        #     is_far_enough = not distance_is_less_than(north, east, down, 1.0)
        #     is_not_vertical = not check_angle_with_threshold((north, east, down), threshold=2)
        #     if is_far_enough and is_not_vertical:
        #         # the drone is headed nearly up or down
        #         # therefore we won't set a yaw
        #         # self.debounce_logger.info("Drone is headed nearly up or down.", 0.5)
        #         self.trajectory.yaw = atan2(north, east)

        # if distance < 1.5:
        #     self.debounce_logger.info("Waypoint reached, but trajectory is not done yet.", 0.5)
        # if self.trajectory.is_done():
        #     self.debounce_logger.info("Trajectory is done, but waypoint is too far away.", 0.5)



from dr_onboard_autonomy.mission_helper import MissionBuilder
from dr_onboard_autonomy.states.components.TrajectoryCommander import TrajectoryCommander
