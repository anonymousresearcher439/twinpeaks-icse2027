import copy
import math
from dr_onboard_autonomy.states.BriarHover import BriarHover
from dr_onboard_autonomy.states.BriarTravel import BriarTravel
import rospy
import time

from typing import Tuple

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    amsl_to_ellipsoid,
    convert_Lla_to_tuple,
    convert_LlaDict_to_tuple,
    convert_tuple_to_Lla,
    convert_tuple_to_LlaDict,
    convert_Lla_to_LlaDict,
    ellipsoid_to_amsl,
    LlaDict,
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, Setpoint

from .BaseState import BaseState

from .BriarCircle import BriarCircle
from .BriarWaypoint import BriarWaypoint


def find_hover_position(stare_position: Lla, pitch: float, distance: float, starting_angle: float) -> Lla:
    """
    Find the hover position.

    Args:
        subject_position is where the camera looks. Alt is Ellipsoidal.
        pitch is in radians
        distance is in meters
        starting_angle is in radians
    """
    horizontal_distance = distance * math.cos(pitch)
    north = horizontal_distance * math.cos(starting_angle)
    east = horizontal_distance * math.sin(starting_angle)
    down = -distance * math.sin(pitch)

    return stare_position.move_ned(north, east, down)

class BetterHover(BaseState):
    def __init__(
        self,
        stare_position: LlaDict,
        pitch: float,
        distance: float,
        starting_angle: float,
        speed:float=5.0,
        hover_time:float = 10.0,
        cruising_altitude:float=29,
        data=None,
        **kwargs,
    ):
        """
        Finds the hover position given pitch, distance, and starting_angle.
        Flies there, then hovers for the specified time.

        Args:
            stare_position: where the camera looks. It's alt is AMSL
            pitch: float in degrees. It's the angle between the horizontal
                plane and the ray that points from the stare_position to the drone
            distance: float in meters. How far the drone is from the stare_position
                when it hovers.
            starting_angle: float in degrees. This is the compass heading
                that points from the stare_position to the hover position.
                A value of 0 means the hover position is to the North.
            speed: float in meters per second. This is how fast the drone will
                fly on its way to the hover position
            hover_time: float in seconds. How long the drone will hover.
            cruising_altitude: how high the drone will fly on it's way to the hover position
        """
        
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_hover", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        self.stare_position = BriarLla(stare_position, is_amsl=True)

        self.hover_position = find_hover_position(
            stare_position=self.stare_position.ellipsoid.lla,
            pitch = math.radians(float(pitch)),
            distance=float(distance),
            starting_angle=math.radians(float(starting_angle))
        )
        self.hover_position = convert_Lla_to_LlaDict(self.hover_position)
        self.hover_position = BriarLla(self.hover_position, is_amsl=False)

        travel_args = copy.copy(kwargs)
        travel_args.update({
            'waypoint': self.hover_position.amsl.dict,
            'stare_position': self.stare_position.amsl.dict,
            'speed': float(speed),
            'cruising_altitude': cruising_altitude,
            'data': data,
            'name': self.name,
        })
        self.travel_state = BriarTravel(**travel_args)

        hover_args = copy.copy(kwargs)
        hover_args.update({
            'hover_time': float(hover_time),
            'stare_position': self.stare_position.amsl.dict,
            'data': data,
            'name': self.name,
        })
        self.hover_state = BriarHover(**hover_args)

        self.data = data

        
    def on_entry(self, userdata):
        travel_outcome = self.travel_state.execute(userdata)
        if travel_outcome != "succeeded_waypoints":
            return travel_outcome
        
        return self.hover_state.execute(userdata)
        

    