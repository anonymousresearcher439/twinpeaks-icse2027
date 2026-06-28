import copy
import math
from typing import List
import time

from numpy import ndarray
import numpy as np

import rospy
from tf.transformations import unit_vector

from droneresponse_mathtools import Lla
from dr_onboard_autonomy.states.BriarHover import BriarHover
from dr_onboard_autonomy.states.BriarTravel import BriarTravel
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
from dr_onboard_autonomy.state_factory import register_state

from .BaseState import BaseState

from .BriarCircle import BriarCircle
from .BriarWaypoint import BriarWaypoint

"""Example JSON:
{
    "name": "BetterPath",
    "class": "BetterPath",
    "args": {
        "path": [
            {
                "latitude": 41.606514240899294,
                "longitude":-86.35654012997003,
                "relative_altitude": 16
            },
            {
                "latitude": 41.60645517660092,
                "longitude":-86.35599550110241,
                "relative_altitude": 20
            },
            {
                "latitude": 41.60656223026213,
                "longitude":-86.35528577901390,
                "relative_altitude": 18
            }
        ],
        "stare_position": {
            "latitude": 41.606879523876685, 
            "longitude": -86.35603187231904,
            "relative_altitude": 1.0
        },
        "speed": 5.0,
        "time_limit": 20.0,
        "init_speed": 7.0
    },
    "transitions": [
        {
            "target": "BriarCircle",
            "condition": "succeeded_waypoints"
        }
    ]
}
"""


def find_direction_of_travel(start: BriarLla, end: BriarLla) -> np.ndarray:
    t = start.ellipsoid.lla.distance_ned(end.ellipsoid.lla)
    return unit_vector(t)

def magnitude(vec: ndarray):
    return math.sqrt(np.dot(vec, vec))

class SetpointAnimator:
    def __init__(self, start_pos: BriarLla, end_pos: BriarLla, speed: float):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.distance = start_pos.ellipsoid.lla.distance(end_pos.ellipsoid.lla)
        self.speed = float(speed)

        self.current_pos = start_pos
        self.start_time = None

        self.direction = find_direction_of_travel(start_pos, end_pos)
        self.velocity = self.direction * self.speed
    
    def next(self):
        if self.start_time is None:
            self.start_time = time.monotonic()
            delta_t = 0
        else:
            delta_t = time.monotonic() - self.start_time

        return self.find_position_setpoint(delta_t)

    
    def find_position_setpoint(self, delta_t: float):
        delta_position = self.velocity * delta_t
        
        if magnitude(delta_position) >= self.distance:
            return self.end_pos.amsl.tup

        result_pos = self.start_pos.ellipsoid.lla.move_ned(*delta_position)
        result_pos = convert_Lla_to_LlaDict(result_pos)
        result_pos = BriarLla(result_pos, is_amsl=False)
        return result_pos.amsl.tup


@register_state(default_transitions={
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
})
class BetterPath(BaseState):
    def __init__(
        self,
        path: List[LlaDict],
        stare_position: LlaDict,
        speed: float = 2.5,
        time_limit: float = 30.0,
        init_speed: float = 10,
        **kwargs,
    ):
        """
        Fly to each location in the list of waypoints. Move through the list
        forwards then backwards repeatedly until the specified amount of time
        has passed.

        The timer starts when the drone arrives at the first waypoint and starts
        flying to the second. This is because the drone could enter
        this state from anywhere and the initial flight to the first waypoint
        follows a calculated path that goes above the trees at Griffiss. See
        BriarTravel.py for details.
        
        The init_speed argument controls how fast the drone flies before it
        starts moving through the list of waypoints. This is how fast the drone
        flies to the first waypoint when it's following the path above the
        trees.

        when flying through the list of waypoints, the drone follows the
        straight line path from waypoint to waypoint.

        Args:
            path: the list of waypoints that the drone will visit over and
                over. This list must contain at least two waypoints. All
                altitudes are specified as meters AMSL.
            stare_position: where the camera looks. The alt is AMSL.
            speed: the speed the drone will fly in meters per second. This is
                how fast the drone flies when it's moving from waypoint to waypoint
            time_limit: how long the drone will follow the path
            init_speed: the speed that the drone will flies before it starts
                moving through the list. The drone must travel from it's
                starting position to the first waypoint going above the trees.
                This value controls how fast the drone flies above the trees.
        """
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        rospy.loginfo(f"BetterPath path = {path}")
        self.stare_position = BriarLla(stare_position, is_amsl=True)
        
        self.first_waypoint = BriarLla(path[0], is_amsl=True)
        travel_args = copy.copy(kwargs)
        travel_args.update({
            'waypoint': self.first_waypoint.amsl.dict,
            'stare_position': self.stare_position.amsl.dict,
            'speed': float(init_speed),
            'data': self.data,
            'name': self.name,
        })
        self.travel_state = BriarTravel(**travel_args)
        

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position)

        self.message_senders.add(RepeatTimer('timer_done', time_limit))
        self.handlers.add_handler('timer_done', self.on_timer)

        self.message_senders.add(RepeatTimer("setpoint_update", 0.05))
        self.handlers.add_handler("setpoint_update", self.on_setpoint_update)

        self.setpoint_driver = Setpoint(self)
        self.gimbal_driver = Gimbal(self)

        self.index = 0
        self.current_pos = None
        self.waypoints = [BriarLla(x, is_amsl=True) for x in path]
        self.speed = float(speed)
        self.animator = None
        
    def on_entry(self, userdata):
        travel_outcome = self.travel_state.execute(userdata)
        if travel_outcome != "succeeded_waypoints":
            return travel_outcome
        
        self.setpoint_driver.start()
        self.setpoint_driver.lla = self.first_waypoint.amsl.tup

        self.gimbal_driver.start()
        self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
    
    def on_timer(self, message):
        return "succeeded_waypoints"
    
    def on_position(self, message):
        pos = message["data"]
        pos = (pos.latitude, pos.longitude, pos.altitude)
        pos = convert_tuple_to_LlaDict(pos)
        self.current_pos = BriarLla(pos, is_amsl=False)

        current_waypoint = self.waypoints[self.index]
        distance_to_waypoint = self.current_pos.ellipsoid.lla.distance(current_waypoint.ellipsoid.lla)
        if distance_to_waypoint < 1.0:
            self.update_waypoint()
        else:
            to_waypoint_ned = self.current_pos.ellipsoid.lla.distance_ned(current_waypoint.ellipsoid.lla) 
    
    def on_setpoint_update(self, message):
        if self.animator is None:
            return
        self.setpoint_driver.lla = self.animator.next()
    
    def update_waypoint(self):
        rospy.loginfo("BetterPath: Moving to next waypoint")

        start_pos = self.waypoints[self.index]

        self.index += 1
        if self.index >= len(self.waypoints):
            self.index = 0
            self.waypoints.reverse()
        
        end_pos = self.waypoints[self.index]
        rospy.loginfo(f"BetterPath: the next waypoint is {end_pos.amsl.tup}")
        self.animator = SetpointAnimator(start_pos, end_pos, self.speed)
