import math
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
    ellipsoid_to_amsl,
    LlaDict,
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, Setpoint

from .BaseState import BaseState


def find_radius(center_position: Lla, start_position: Lla) -> float:
    """Given the center position and the start position find the radius"""
    # find the location above or below the starting point that sits on North East plane with the center_position
    ned_vec = center_position.distance_ned(start_position)
    horizontal_position = center_position.move_ned(ned_vec[0], ned_vec[1], 0.0)

    return center_position.distance(horizontal_position)


def find_starting_theta(center_position: Lla, start_position: Lla) -> float:
    """Given center pos and starting pos, find starting angle.
    This is the same as the compass heading that points from center position to starting position
    """

    ned_vec = center_position.distance_ned(start_position)
    n, e = ned_vec[0], ned_vec[1]

    return math.atan2(e, n)


def find_delta_theta(delta_t: float, speed: float, radius: float) -> float:
    arc_distance = speed * delta_t
    delta_theta = arc_distance / radius
    return delta_theta


def find_waypoint(
    center_position: Lla,
    start_position: Lla,
    sweep_angle_radians: float,
    speed: float,
    delta_t: float,
) -> Lla:
    radius = find_radius(center_position, start_position)
    theta0 = find_starting_theta(center_position, start_position)

    delta_theta = find_delta_theta(delta_t, speed, radius)

    if delta_theta > abs(sweep_angle_radians):
        delta_theta = abs(sweep_angle_radians)

    if sweep_angle_radians < 0:
        delta_theta = -delta_theta

    theta = theta0 + delta_theta

    north = radius * math.cos(theta)
    east = radius * math.sin(theta)

    horizontal_pos = center_position.move_ned(north, east, 0)
    return Lla(horizontal_pos.lat, horizontal_pos.lon, start_position.alt)


def find_endpoint(
    center_position: Lla, start_position: Lla, sweep_angle_radians: float
) -> Lla:
    radius = find_radius(center_position, start_position)
    theta0 = find_starting_theta(center_position, start_position)
    theta = theta0 + sweep_angle_radians

    north = radius * math.cos(theta)
    east = radius * math.sin(theta)
    horizontal_pos = center_position.move_ned(north, east, 0)
    return Lla(horizontal_pos.lat, horizontal_pos.lon, start_position.alt)


class BriarCircle(BaseState):
    def __init__(
        self,
        center_position: LlaDict,
        stare_position: LlaDict,
        sweep_angle: float = 360,
        speed: float = 2.0,
        data=None,
        **kwargs,
    ):
        """
        Fly in a circle while pointing the camera
        The drone's starting location is considered the starting point.
        The radius of the circle is calculated from the starting location and the center point

        Args:
            center_position has its altitude as AMSL
            stare_position has its alt as AMSL
            sweep_angle is in degrees
            speed is in meters per second
        """

        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_circle", "error", "human_control", "abort"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        self.data = data

        self.center_position_amsl_tup = convert_LlaDict_to_tuple(center_position)
        self.center_position_ellipsoidal_tup = amsl_to_ellipsoid(
            self.center_position_amsl_tup
        )
        self.center_position_ellipsoidal_lla = convert_tuple_to_Lla(
            self.center_position_ellipsoidal_tup
        )

        self.sweep_angle = math.radians(sweep_angle)
        self.speed = float(speed)

        self.stare_position = BriarLla(stare_position, is_amsl=True)

        self.setpoint_driver = Setpoint(self)
        self.gimbal_driver = Gimbal(self)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)
        self.handlers.add_handler("position", self.on_first_position_message)

        self.message_senders.add(RepeatTimer("waypoint_update", 0.05))
        self.handlers.add_handler("waypoint_update", self.update_waypoint)

        self.start_time = time.monotonic()
        self.start_position_ellipsoidal_lla = None
        self.end_position_ellipsoidal_lla = None

    def on_entry(self, userdata):
        self.setpoint_driver.start()
        self.gimbal_driver.start()
        self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup

    def on_first_position_message(self, message):
        if self.start_position_ellipsoidal_lla is not None:
            return
        pos = message["data"]
        self.start_position_ellipsoidal_lla = Lla(
            pos.latitude, pos.longitude, pos.altitude
        )
        self.start_time = time.monotonic()
        self.end_position_ellipsoidal_lla = find_endpoint(
            self.center_position_ellipsoidal_lla,
            self.start_position_ellipsoidal_lla,
            self.sweep_angle,
        )

    def update_waypoint(self, message):
        if self.start_position_ellipsoidal_lla is None:
            return

        delta_t = time.monotonic() - self.start_time

        waypoint_ellipsoidal_lla = find_waypoint(
            center_position=self.center_position_ellipsoidal_lla,
            start_position=self.start_position_ellipsoidal_lla,
            sweep_angle_radians=self.sweep_angle,
            speed=self.speed,
            delta_t=delta_t,
        )
        waypoint_ellipsoidal_tup = convert_Lla_to_tuple(waypoint_ellipsoidal_lla)
        waypoint_amsl_tup = ellipsoid_to_amsl(waypoint_ellipsoidal_tup)
        self.setpoint_driver.lla = waypoint_amsl_tup

    def on_position_message(self, message):
        if self.end_position_ellipsoidal_lla is None:
            return

        pos = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)

        distance = current_pos.distance(self.end_position_ellipsoidal_lla)

        delta_t = time.monotonic() - self.start_time
        radius = find_radius(self.center_position_ellipsoidal_lla, self.start_position_ellipsoidal_lla)
        delta_theta = find_delta_theta(delta_t, self.speed, radius)
        
        if distance < 1.0 and delta_theta >= self.sweep_angle:
            return "succeeded_circle"