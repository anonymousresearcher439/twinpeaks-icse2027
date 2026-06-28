import copy
import math
import rospy
import time

from typing import Tuple

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    convert_Lla_to_tuple,
    convert_LlaDict_to_tuple,
    convert_tuple_to_Lla,
    ellipsoid_to_amsl,
    LlaDict,
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, Setpoint
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState

from .BriarCircle import BriarCircle
from .BriarTravel import BriarTravel


"""Example JSON:
{
    "name": "BetterCircle",
    "args": {
        "center_position": {
            "latitude": 41.6064512009611, 
            "longitude": -86.355995270215,
            "relative_altitude": 0.0
        },
        "stare_position": {
            "latitude": 41.6064512009611, 
            "longitude": -86.355995270215,
            "relative_altitude": 0.0
        },
        "pitch": 50.0,
        "distance": 20.0,
        "starting_angle": 180.0,
        "sweep_angle": 60.0,
        "speed": 5.0
    },
    "transitions": [
        {
            "target": "BetterHover",
            "condition": "succeeded_circle"
        }
    ]
}
"""


def find_start_position(center_position: Lla, pitch: float, distance: float, starting_angle: float) -> Lla:
    """
    Finds the place we go to start flying in a circle

    Args:
        center_position is the center of the circle
        pitch is in radians
        distance is in meters
        starting_angle is in radians
    """
    horizontal_distance = distance * math.cos(pitch)
    north = horizontal_distance * math.cos(starting_angle)
    east = horizontal_distance * math.sin(starting_angle)
    down = -distance * math.sin(pitch)

    return center_position.move_ned(north, east, down)


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class BetterCircle(BaseState):
    def __init__(
        self,
        center_position: LlaDict,
        stare_position: LlaDict,
        pitch: float,
        distance: float,
        starting_angle: float,
        sweep_angle: float,
        speed:float=2.0,
        **kwargs,
    ):
        """
        Finds the starting position, flies there, then fly in a circle

        Args:
            center_position the point that the drone will circle around.
                The altitude is AMSL.
            stare_position: where the camera looks. It's alt is AMSL
            pitch - a float in degrees and it's the angle from the horizontal
                plane to the ray that points from the center position to the drone
            distance - a float is how far the drone is from the center position
                when it starts
            starting_angle - a float in degrees. This is the compass heading
                that points from the center position to the starting position.
                A value of 0 means the starting position is to the North.
            sweep_angle: float in degrees. This is how much of the circle the
                drone should fly. A positive value causes the drone to fly
                clockwise as seen from above.
            speed: float in meters per second. This is how fast the drone will fly 
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_circle"], STANDARD_TRANSITIONS_FLYING.keys())
        super().__init__(**kwargs)

        self.center_position_amsl_tup = convert_LlaDict_to_tuple(center_position)
        self.center_position_ellipsoidal_tup = amsl_to_ellipsoid(
            self.center_position_amsl_tup
        )
        self.center_position_ellipsoidal_lla = convert_tuple_to_Lla(
            self.center_position_ellipsoidal_tup
        )

        self.pitch = math.radians(pitch)
        self.distance = float(distance)
        self.starting_angle = math.radians(starting_angle)

        self.sweep_angle = math.radians(sweep_angle)
        self.speed = float(speed)

        self.starting_pos_ellipsoidal_lla = find_start_position(
            center_position=self.center_position_ellipsoidal_lla,
            pitch=self.pitch,
            distance=self.distance,
            starting_angle=self.starting_angle,
        )

        self.starting_pos_ellipsoidal_tup = convert_Lla_to_tuple(
            self.starting_pos_ellipsoidal_lla
        )
        self.starting_pos_amsl_tup = ellipsoid_to_amsl(
            self.starting_pos_ellipsoidal_tup
        )

        waypoint_args = copy.copy(kwargs)
        waypoint_args.update({
            "waypoint": {
                "latitude": self.starting_pos_amsl_tup[0],
                "longitude": self.starting_pos_amsl_tup[1],
                "altitude": self.starting_pos_amsl_tup[2],
            },
            "stare_position": stare_position,
            "speed": self.speed,
            "data": self.data,
            "name": self.name,
        })
        self.waypoint_state = BriarTravel(**waypoint_args)

        circle_args = copy.copy(kwargs)
        circle_args.update({
            "center_position": center_position,
            "stare_position": stare_position,
            "sweep_angle": sweep_angle,
            "speed": self.speed,
            "data": self.data,
            "name": self.name,
        })
        self.circle_state = BriarCircle(**circle_args)
        
    def on_entry(self, userdata):
        waypoint_result = self.execute_substate(self.waypoint_state, userdata)
        if waypoint_result != "succeeded_waypoints":
            return waypoint_result
        
        return self.execute_substate(self.circle_state, userdata)
        

    