import copy
import math
from dr_onboard_autonomy.states.BriarHover import BriarHover
from dr_onboard_autonomy.states.BriarTravel import BriarTravel

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    convert_Lla_to_LlaDict,
    LlaDict,
)
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState

"""Example JSON:
{
    "name": "BetterHover",
    "class": "BetterHover",
    "args": {
        "stare_position": {
            "latitude": 41.606453922600465,  
            "longitude": -86.35555549206501,
            "relative_altitude": 1.0
        },
        "pitch": 45.0,
        "distance": 20.0,
        "starting_angle": 270.0,
        "speed": 5.0,
        "hover_time": 7.0,
        "cruising_altitude": 24.0
    },
    "transitions": [
        {
            "target": "BetterPath",
            "condition": "succeeded_hover"
        }
    ]
}
"""

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


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
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
            cruising_altitude: how high the drone will fly on it's way to the
                hover position. Altitude is in meters above ground level where
                ground level is determined by the home position.
        """
        outcomes_arg = kwargs.get("outcomes", [])
        outcomes_set = set(outcomes_arg)
        default_outcomes = set()

        outcomes_set |= default_outcomes

        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_hover"], STANDARD_TRANSITIONS_FLYING.keys())
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
            'data': self.data,
            'name': self.name,
        })
        self.travel_state = BriarTravel(**travel_args)

        hover_args = copy.copy(kwargs)
        hover_args.update({
            'hover_time': float(hover_time),
            'stare_position': self.stare_position.amsl.dict,
            'data': self.data,
            'name': self.name,
        })
        self.hover_state = BriarHover(**hover_args)

        

        
    def on_entry(self, userdata):
        travel_outcome = self.execute_substate(self.travel_state, userdata)
        if travel_outcome != "succeeded_waypoints":
            return travel_outcome
        
        return self.execute_substate(self.hover_state, userdata)
        

    