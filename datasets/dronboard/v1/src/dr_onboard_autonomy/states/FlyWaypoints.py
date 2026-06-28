from dataclasses import dataclass
from enum import IntEnum
import time
from typing import List, TypedDict
from dr_onboard_autonomy.states.components.Gimbal import Gimbal
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
import rospy
from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState

from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, convert_tuple_to_LlaDict

import tf
from threading import Thread
from dr_onboard_autonomy.gimbal import MavlinkNode
import math


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
        if "velocity" in waypoint:
            waypoint["speed"] = waypoint["velocity"]
        if "speed" not in waypoint:
            waypoint["speed"] = default_speed
        wp = _Waypoint(position=BriarLla(waypoint), speed=waypoint["speed"])
        result.append(wp)
    return result


class FlyWaypoints(BaseState):
    def __init__(self, data=None, waypoints:List[WaypointDict]=[], stare_position:LlaDict=None, default_speed=2.24, **kwargs):
        outcomes = []
        if "outcomes" in kwargs:
            outcomes = kwargs["outcomes"]

        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort", "found", "rtl"]:

            if other_outcome not in outcomes:
                outcomes.append(other_outcome)
        kwargs["outcomes"] = outcomes
        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        self.data = data
        
        self.index = 0
        self.waypoints: List[_Waypoint] = build_waypoints(waypoints, default_speed)

        self.gimbal_driver = Gimbal(self)

        if stare_position is None and 'target_gps_location' in kwargs:
            stare_position = kwargs['target_gps_location']
        if stare_position is not None:
            self.stare_position = BriarLla(stare_position)

        if "found" in outcomes:
            self.message_senders.add(self.reusable_message_senders.find('vision'))
            self.handlers.add_handler("vision", self.on_vision_message)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)
        self._arrival_time = None

    def on_entry(self, userdata):
        self.index = 0
        self._arrival_time = None
        outcome = self.update_waypoint()
        if outcome:
            return outcome

        self.trajectory.start()
        self.gimbal_driver.start()

        if self.stare_position is None:
            self.gimbal_driver.stare_direction = (0, -90, 0)
        else:
            self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
    
    def update_waypoint(self):
        if self.index == len(self.waypoints):
            return "succeeded_waypoints"
        
        wp = self.waypoints[self.index]
        self._arrival_time = None
        self.trajectory.fly_to_waypoint(wp.position.amsl.lla, wp.speed)
        return None

    def on_vision_message(self, message):
        return "found"

    def on_position_message(self, message):
        pos = message["data"]
        pos = pos.latitude, pos.longitude, pos.altitude
        pos = convert_tuple_to_LlaDict(pos)
        pos = BriarLla(pos, is_amsl=False)
        
        waypoint = self.waypoints[self.index].position
        distance = pos.ellipsoid.lla.distance(waypoint.ellipsoid.lla)

        if distance < 1.0 and self.trajectory.is_done():
            self.index += 1
            outcome = self.update_waypoint()
            if outcome:
                return outcome

