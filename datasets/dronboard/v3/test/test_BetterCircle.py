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

#from dr_onboard_autonomy.states import BaseState, BriarCircle, MessageHandler
from dr_onboard_autonomy.states.BetterCircle import find_start_position
from dr_onboard_autonomy.states.components import Setpoint


class TestBetterCircle(unittest.TestCase):
    def test_find_start_position(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = math.radians(45)
        distance = 50
        starting_angle = math.radians(180)

        actual_start_pos = find_start_position(center_position, pitch, distance, starting_angle)
        
        horizontal_displacement = math.cos(pitch) * distance
        down = -distance * math.sin(pitch)
        expected_start_pos = center_position.move_ned(-horizontal_displacement, 0, down)

        self.assertAlmostEqual(actual_start_pos.distance(expected_start_pos), 0)
    
    def test_find_start_position_when_pitch_is_zero(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = 0
        distance = 50
        starting_angle = math.radians(90)

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(0, 50, 0)
        
        self.assertAlmostEqual(actual.distance(expected), 0)

    def test_find_start_position_when_pitch_is_zero2(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = 0
        distance = 100 * math.sqrt(2)
        starting_angle = math.radians(45)

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(100, 100, 0)
        
        self.assertAlmostEqual(actual.distance(expected), 0)
    
    def test_find_start_position_when_pitch_is_zero3(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = 0
        distance = 100 * math.sqrt(2)
        starting_angle = math.radians(270 + 45) # north west

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(100, -100, 0)
        
        self.assertAlmostEqual(actual.distance(expected), 0)

    def test_find_start_position_when_pitch_is_zero_and_start_angle_is_west(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = 0
        distance = 100
        starting_angle = math.radians(270) # west

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(0, -100, 0)
        
        self.assertAlmostEqual(actual.distance(expected), 0)

    def test_find_start_position_when_pitch_is_45_degrees(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = math.radians(45)
        distance = 100*math.sqrt(2)
        starting_angle = math.radians(270) # west

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(0, -100, -100)
        
        self.assertAlmostEqual(actual.distance(expected), 0)

    def test_find_start_position_when_pitch_is_45_degrees2(self):
        center_position: Lla = Lla(41.70573086523541, -86.24421841999177, 218.566)
        pitch = math.radians(45)
        distance = 100*math.sqrt(2)
        starting_angle = math.radians(180) # west

        actual = find_start_position(center_position, pitch, distance, starting_angle)
        expected = center_position.move_ned(-100, 0, -100)
        
        self.assertAlmostEqual(actual.distance(expected), 0)