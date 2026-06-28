from typing import Tuple, TypedDict
import time
import rospy
from tf.transformations import unit_vector
from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.briar_helpers import LlaDict, convert_LlaDict_to_tuple, amsl_to_ellipsoid, ellipsoid_to_amsl
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, Setpoint

from .BaseState import BaseState


def find_waypoint(current_pos: Lla, target_pos: Lla, speed: float, delta_t: float) -> Lla:
    direction = current_pos.distance_ned(target_pos)
    direction = unit_vector(direction)

    distance = speed * delta_t
    # if we're going to overshoot the target, then snap to the target
    if distance > current_pos.distance(target_pos):
        return float(target_pos.latitude), float(target_pos.longitude), float(target_pos.altitude)
    
    displacement = direction * distance
    delta_n, delta_e, delta_d = displacement[0], displacement[1], displacement[2]
    
    waypoint = current_pos.move_ned(delta_n, delta_e, delta_d)
    return float(waypoint.latitude), float(waypoint.longitude), float(waypoint.altitude)


class BriarWaypoint(BaseState):
    def __init__(self, data=None, waypoint:LlaDict=None, stare_position:LlaDict=(0.,0.,0.), speed:float=2.0, **kwargs):
        """
        Fly to the specified waypoint taking the most direct path.

        Args:
            waypoint: the final destination. Alt is AMSL
            stare_position: where the camera should look. Alt is AMSL
            speed: how fast to fly in meters per second
        """
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        waypoint = convert_LlaDict_to_tuple(waypoint)
        self.waypoint_amsl = waypoint
        self.waypoint_ellipsoidal = amsl_to_ellipsoid(waypoint)
        waypoint_lat, waypoint_lon, waypoint_alt = self.waypoint_ellipsoidal
        self.target_lla = Lla(waypoint_lat, waypoint_lon, waypoint_alt)

        stare_position = convert_LlaDict_to_tuple(stare_position)
        stare_position = amsl_to_ellipsoid(stare_position)
        self.stare_position = stare_position
        self.speed=float(speed)

        self.data = data

        self.setpoint_driver = Setpoint(self)
        self.gimbal_driver = Gimbal(self)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)
        self.handlers.add_handler("position", self.on_first_position_message)

        self.message_senders.add(RepeatTimer("waypoint_update", 0.05))
        self.handlers.add_handler("waypoint_update", self.update_waypoint)
        self.start_pos = None
        self.start_time = time.monotonic()
    
    def on_entry(self, userdata):
        self.setpoint_driver.start()
        self.gimbal_driver.start()
        self.gimbal_driver.stare_position = self.stare_position
    
    def on_first_position_message(self, message):
        if self.start_pos is not None:
            return
        pos = message["data"]
        self.start_pos = Lla(pos.latitude, pos.longitude, pos.altitude) # Ellipsoidal altitude
        self.start_time = time.monotonic()
    
    def on_position_message(self, message):
        pos = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        
        distance = current_pos.distance(self.target_lla)
        if distance < 1.0:
            return "succeeded_waypoints"
    
    def update_waypoint(self, message):
        if self.start_pos is None:
            return
        delta_t = time.monotonic() - self.start_time
        waypoint_ellipsoidal = find_waypoint(self.start_pos, self.target_lla, self.speed, delta_t)
        waypoint_amsl = ellipsoid_to_amsl(waypoint_ellipsoidal)
        self.setpoint_driver.lla = waypoint_amsl

