import copy
import math
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
import rospy
import time

from dr_onboard_autonomy.briar_helpers import (
    convert_tuple_to_LlaDict,
    LlaDict,
    BriarLla
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, Setpoint

from .BaseState import BaseState
from .BriarWaypoint import BriarWaypoint


class BriarTravel(BaseState):
    def __init__(
        self,
        waypoint:LlaDict,
        stare_position:LlaDict=None,
        cruising_altitude=29.0,
        speed:float=5.0,
        data=None, 
        **kwargs,
    ):
        """
        Flies the drone to the specified waypoint, taking a path that goes over
        the trees. Here's how:

        1. The drone flies up (or down) to the crusing_altitude.
        2. It flies to the waypoint's latitude and longitude while staying at
            the cruising altitude.
        3. It flies down (or up) to the waypoint

        Args:
            waypoint: the drone's final destination. The altitude is AMSL.
            stare_position: where the camera looks. It's alt is AMSL
            cruising_altitude: The altitude that the drone maintains during
                horizontal traversal. Specified as meters above ground level,
                where ground level is determined by the location where the drone
                was armed.
            speed: how fast the drone will fly in meters per second. 
        """
        
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        self.waypoint = BriarLla(waypoint, is_amsl=True)

        if stare_position is None:
            stare_position = waypoint
        self.stare_position = BriarLla(stare_position, is_amsl=True)
        
        self.cruising_altitude = cruising_altitude
        self.speed = float(speed)
        self.data = data
        self.kwargs = kwargs
        
    def on_entry(self, userdata):
        home = self.data["arm_position"]
        home = home.latitude, home.longitude, home.altitude
        home = convert_tuple_to_LlaDict(home)
        home = BriarLla(home, is_amsl=False)
        move_alt_amsl = home.amsl.lla.alt + self.cruising_altitude

        drone: MAVROSDrone = self.drone
        location = drone.data.location
        wp1 = location['latitude'], location['longitude'], move_alt_amsl
        wp1 = convert_tuple_to_LlaDict(wp1)
        wp1 = BriarLla(wp1, is_amsl=True)

        wp2 = self.waypoint.amsl.lla
        wp2 = wp2.lat, wp2.lon, move_alt_amsl
        wp2 = convert_tuple_to_LlaDict(wp2)
        wp2 = BriarLla(wp2, is_amsl=True)

        wp3 = self.waypoint

        waypoints = [wp1, wp2, wp3]

        for wp in waypoints:
            args = copy.copy(self.kwargs)
            args.update({
                'waypoint': wp.amsl.dict,
                'stare_position': self.stare_position.amsl.dict,
                'speed': self.speed,
                'data': copy.copy(self.data)
            })
            fly_waypoint_state = BriarWaypoint(**args)
            
            result = fly_waypoint_state.execute(userdata)
            if result != "succeeded_waypoints":
                return result
        
        return "succeeded_waypoints"
