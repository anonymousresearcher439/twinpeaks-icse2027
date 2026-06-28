from dataclasses import dataclass
from typing import List, TypedDict, Union

from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict
from dr_onboard_autonomy.states import BriarWaypoint, Waypoint
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState


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
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_waypoints"], STANDARD_TRANSITIONS_FLYING.keys())

        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)
        self.mission_builder = kwargs["mission_builder"]

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
        index = 0
        while index < len(self.waypoints):
            waypoint = self.waypoints[index]
            waypoint_state = self._make_Waypoint(waypoint, index)
            outcome = self.execute_substate(waypoint_state, userdata)
            if outcome == "deadlock":
                diversion_waypoint = self._create_diversion_waypoint()
                self.waypoints.insert(index, diversion_waypoint)
                continue
            if outcome != "succeeded_waypoints":
                return outcome
            index += 1
        return "succeeded_waypoints"

    def _make_Waypoint(self, waypoint: _Waypoint, index: int) -> Waypoint:
        args = {
            'waypoint': waypoint.position.amsl.dict,
            'speed': waypoint.speed,
        }
        args.update(self.kwargs)
        args = self.mission_builder.build_kwargs(args)
        outcomes = self.args.get('outcomes', [])
        outcomes.append('deadlock')
        args['outcomes'] = outcomes

        if 'name' not in args:
            args['name'] = self.name

        if self.stare_position is not None:
            args['stare_position'] = self.stare_position.amsl.dict

        if self.stare_pitch is not None:
            args['stare_pitch'] = self.stare_pitch

        return Waypoint(**args)

    def _create_diversion_waypoint(self) -> _Waypoint:
        cur_pos = self.drone.data.location.get_position()
        cur_pos = cur_pos.to_wgs84_ellipsoid()
        cur_pos = BriarLla.from_args(cur_pos.latitude, cur_pos.longitude, cur_pos.altitude, is_amsl=False)

        home: WaypointDict = self.kwargs['home'] # in AMSL
        home = BriarLla(home, is_amsl=True)
        
        max_alt = home.ellipsoid.lla.altitude + 100
        diversion_wgs84 = cur_pos.ellipsoid.lla.move_ned(0, 0, -22)
        diversion_alt = min(diversion_wgs84.altitude, max_alt)
        
        diversion_pos = BriarLla.from_args(diversion_wgs84.latitude, diversion_wgs84.longitude, diversion_alt, is_amsl=False)
        waypoint_dict = diversion_pos.amsl.dict
        return build_waypoints([waypoint_dict], self.default_speed)[0]
