from dataclasses import dataclass
from typing import List, TypedDict, Union
from dr_onboard_autonomy.states.components.Gimbal import Gimbal
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
import rospy
from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states import BriarWaypoint


from .BaseState import BaseState

from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, convert_tuple_to_LlaDict

import math
import tf
from threading import Thread
from dr_onboard_autonomy.gimbal import MavlinkNode
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


"""Example JSON:

{
    "name": "FlyWaypoints",
    "class": "FlyWaypoints",
    "args": {
        "waypoints": [
            {"latitude": 41.606492332962360, "longitude": -86.35584795401677, "relative_altitude": 15},
            {"latitude": 41.606470291689604, "longitude": -86.35553704404666, "relative_altitude": 15},
            {"latitude": 41.606652778945936, "longitude": -86.35523126676622, "relative_altitude": 15}
        ],
        "default_speed": 5.0,
        "stare_pitch": 45
    },
    "transitions": [
        {
            "target": "Fly Path AMSL",
            "condition": "succeeded_waypoints"
        }
    ]
}

"""


class WaypointDict(TypedDict):
    """
    latitude: degrees
    longitude: degrees
    altitude: meters AMSL
    speed: meters per second
    """
    latitude: float
    longitude: float
    altitude: float
    speed: float


@dataclass(frozen=True, eq=True)
class _Waypoint:
    position: BriarLla
    speed: float


def build_waypoints(waypoints:List[WaypointDict], default_speed):
    result = list()
    for waypoint in waypoints:
        if "velocity" in waypoint and "speed" not in waypoints:
            waypoint["speed"] = waypoint["velocity"]
        if "speed" not in waypoint:
            waypoint["speed"] = default_speed
        wp = _Waypoint(position=BriarLla(waypoint), speed=waypoint["speed"])
        result.append(wp)
    return result


@register_state("FlyWaypoints", "FlyWaypointsAMSL", default_transitions=STANDARD_TRANSITIONS_FLYING)
class FlyWaypoints(BaseState):

    def __init__(self,
                 waypoints: Union[WaypointDict, List[WaypointDict]],
                 default_speed: float = 2.24,
                 stare_position: LlaDict = None,
                 stare_pitch: float = None,
                 **kwargs):
        """
        Fly to the specified list of waypoints taking the most direct path.

        Args:
            waypoints (WaypointDict | List[WaypointDict]): A waypoint or a list of waypoints. Each waypoint is a dictionary with the following keys:
                - latitude (float): Latitude in degrees.
                - longitude (float): Longitude in degrees.
                - altitude (float): Altitude in meters AMSL.
                - speed (float, optional): Speed in meters per second.
                - velocity (float, optional): Alias for speed. If both `speed` and `velocity` are present, `speed` takes priority.
            
            default_speed (float, optional): Speed to fly in meters per second. Used if a waypoint does not specify a speed. Defaults to 2.24 m/s.

            stare_position (LlaDict, optional): A dictionary specifying where the camera should look. It has the following keys:
                - latitude (float): Latitude in degrees.
                - longitude (float): Longitude in degrees.
                - altitude (float): Altitude in meters AMSL.

            stare_pitch (float, optional): Controls how far down to pitch the camera in degrees. Specifically, it represents the rotation in degrees around the camera's LEFT axis. For example, a positive number like 45° makes the camera look down by 45°. If both `stare_pitch` and `stare_position` are set, `stare_position` takes priority.
        """
        outcomes_set = {
            "succeeded_waypoints",
            "error",
            "human_control",
            "abort",
            "rtl"
        }
        if "outcomes" in kwargs:
            outcomes_set.update(kwargs["outcomes"])
        kwargs["outcomes"] = list(outcomes_set)

        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        if isinstance(waypoints, dict):
            waypoints = [waypoints]
        self.waypoints: List[_Waypoint] = build_waypoints(waypoints, default_speed)

        self.stare_position = stare_position
        if self.stare_position is None and 'target_gps_location' in kwargs:
            self.stare_position = kwargs['target_gps_location']
        if self.stare_position is not None:
            self.stare_position = BriarLla.from_dict(self.stare_position, is_amsl=True)
        
        self.stare_pitch = stare_pitch
        self.default_speed = default_speed
        self.kwargs = kwargs

    def on_entry(self, userdata):
        for index, waypoint in enumerate(self.waypoints):
            waypoint_state = self._make_BriarWaypoint(waypoint, index)
            outcome = waypoint_state.execute(userdata=userdata)
            if outcome != "succeeded_waypoints":
                return outcome
        return "succeeded_waypoints"

    def _make_BriarWaypoint(self, waypoint: _Waypoint, index: int) -> BriarWaypoint:
        args = {
            'waypoint': waypoint.position.amsl.dict,
            'speed': waypoint.speed,
        }
        args.update(self.kwargs)

        if 'name' not in args:
            args['name'] = f"{self.name} ({index + 1} of {len(self.waypoints)})"

        if self.stare_position is not None:
            args['stare_position'] = self.stare_position.amsl.dict

        if self.stare_pitch is not None:
            args['stare_pitch'] = self.stare_pitch

        return BriarWaypoint(**args)
