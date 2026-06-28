#!/usr/bin/env python3

import unittest
import math
from unittest.mock import NonCallableMock, PropertyMock
from threading import Event
from queue import Empty, Queue

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.drone_data import Data
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)

# from dr_onboard_autonomy.states import BaseState, BriarCircle, MessageHandler
# from dr_onboard_autonomy.states.components import Setpoint

@unittest.skip("This tests functions that no longer exist. The circle logic was moved to the trajectory module.")
# These are the functions that no longer exist:
# from dr_onboard_autonomy.states.BriarCircle import find_endpoint, find_radius, find_starting_theta, find_delta_theta, find_waypoint
class TestCircle(unittest.TestCase):
    def test_find_radius(self):
        # test the most basic usage
        center = Lla(0,0,0)
        starting = center.move_ned(0, 10, 0)

        R = find_radius(center, starting)

        self.assertAlmostEqual(R, 10)
    
    def test_find_radius2(self):
        # test many inputs
        center = Lla(41.70573086523541, -86.24421841999177, 218.566)

        # (north, east, down, expected_radius)
        starting_positions = [
            (3, 4, 100, 5),
            (10, 0, -40, 10),
            (1, 1, 0, math.sqrt(2)),
            (10, 10, math.pi*10, math.sqrt(2)*10)
        ]

        for t in starting_positions:
            north, east, down, radius = t
            starting = center.move_ned(north, east, down)
            R = find_radius(center, starting)
            self.assertAlmostEqual(radius, R)
    
    def test_find_starting_theta(self):
        center = Lla(41.70573086523541, -86.24421841999177, 218.566)
        starting = center.move_ned(0, 10, -30) # a position directly east should be at 90 degrees
        theta = find_starting_theta(center, starting)
        expected_theta = math.radians(90)
        self.assertAlmostEqual(theta, expected_theta)
    
    def test_find_starting_theta2(self):
        center = Lla(41.70573086523541, -86.24421841999177, 218.566)

        #(north, east, down, expected_degrees)
        starting_list = [
            (10, 10, 10, 45),
            (10, -10, 10, -45),
            (-10, 10, 10, 135),
            (-10, -10, 10, -135),
            (-10, 0, 10, 180),
            (0, -10, 10, -90),
        ]

        for t in starting_list:
            n, e, d, expected_degrees = t
            start = center.move_ned(n, e, d)
            expected_theta = math.radians(expected_degrees)
            actual_theta = find_starting_theta(center, start)
            self.assertAlmostEqual(actual_theta, expected_theta)
    
    def test_find_delta_theta(self):
        # go halfway around the circle
        delta_t = 1 # second
        speed = math.pi # meters per second
        radius = 1 # meter
        # if you go pi meters per second for 1 second you will go pi meters
        # if the radius is 1 then pi meters around the circle is 180 degrees
        actual = find_delta_theta(delta_t, speed, radius)

        expected = math.radians(180)
        self.assertAlmostEqual(actual, expected)
    
    def test_find_delta_theta2(self):
        # go all the way around the circle
        delta_t = 1 # second
        speed = 2*math.pi # meters per second
        radius = 1 # meter
        
        actual = find_delta_theta(delta_t, speed, radius)
        expected = math.radians(360)
        self.assertAlmostEqual(actual, expected)

    def test_find_delta_theta3(self):
        # go 0% around the circle
        delta_t = 0 # second
        speed = 100 # meters per second
        radius = 50 # meter
        
        actual = find_delta_theta(delta_t, speed, radius)
        expected = math.radians(0)
        self.assertAlmostEqual(actual, expected)
    
    def test_find_delta_theta4(self):
        # go part way around the circle
        delta_t = 100 # second
        speed = 4 # meters per second
        radius = 75 # meter
        
        actual = find_delta_theta(delta_t, speed, radius)
        expected = 400 / 75
        self.assertAlmostEqual(actual, expected)
    
    def test_find_waypoint(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        start_position: Lla = center_position.move_ned(30, 0, -40)
        sweep_angle = 45.0
        speed = math.pi * 5
        delta_t = 0.0

        actual_waypoint = find_waypoint(center_position, start_position, sweep_angle, speed, delta_t)
        expected_waypoint = start_position

        self.assertAlmostEqual(actual_waypoint.distance(expected_waypoint), 0.0, 2) # check that we're within 1 cm

    def test_find_waypoint_snaps_to_the_end_position(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        start_position: Lla = center_position.move_ned(30, 0, -40)
        sweep_angle = math.radians(90.0)
        speed = 1
        # the arc distance is about 47.12, so lets make sure delta_t is big enough to move farther 
        delta_t = 47.12 * 1.25

        actual_waypoint = find_waypoint(center_position, start_position, sweep_angle, speed, delta_t)

        # the expected position is calculated
        expected_waypoint = center_position.move_ned(0, 30, 0)
        expected_waypoint = Lla(expected_waypoint.latitude, expected_waypoint.longitude, start_position.altitude)

        self.assertAlmostEqual(actual_waypoint.distance(expected_waypoint), 0.0, 2) # check that we're within 1 cm



    def test_find_endpoint(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        start_position: Lla = center_position.move_ned(30, 0, -40)
        sweep_angle = math.radians(90.0)
        speed = 1
        # the arc distance is about 47.12

        actual_endpoint = find_endpoint(center_position, start_position, sweep_angle)

        # the expected position is calculated
        expected_endpoint = center_position.move_ned(0, 30, 0)
        expected_endpoint = Lla(expected_endpoint.latitude, expected_endpoint.longitude, start_position.altitude)

        self.assertAlmostEqual(actual_endpoint.distance(expected_endpoint), 0.0, 2) # check that we're within 1 cm
