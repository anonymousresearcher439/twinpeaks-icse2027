import inspect
import math
import os
import pathlib
import json
from typing import Any, List
import unittest
from unittest.mock import NonCallableMock, call, patch

from dr_onboard_autonomy.models.config import CameraConfig
from dr_onboard_autonomy.state_factory import STANDARD_TRANSITIONS_FLYING
from tf.transformations import (
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
)
from droneresponse_mathtools import Lla
from pyquaternion import Quaternion as Quat

from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition, Quaternion
from dr_onboard_autonomy.air_lease_service import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import BatteryMonitor
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from dr_onboard_autonomy.gimbal.geolocation import GimbalLimits, TwoAxisGimbalQuatCalculator


from test.mock_types import mock_base_state, mock_copter_drone, mock_mqtt_message, mock_state_kwargs, mock_message, mock_battery_data



class TestBatteryMonitor(unittest.TestCase):
    def test_the_init_method(self):
        outcomes = list(STANDARD_TRANSITIONS_FLYING.keys())
        state = mock_base_state({
            "outcomes": outcomes,
            "trajectory_class": None,
        })
        self.assertIsInstance(state.battery_monitor, BatteryMonitor)

    def test_on_battery_message(self):
        outcomes = list(STANDARD_TRANSITIONS_FLYING.keys())
        state = mock_base_state({
            "outcomes": outcomes,
            "trajectory_class": None,
        })
        
        battery = mock_battery_data(14, 5, 0.99)
        outcome = state.battery_monitor.on_battery(mock_message("battery", battery))
        self.assertIsNone(outcome)
    
    def test_on_battery_message_low_battery(self):
        outcomes = list(STANDARD_TRANSITIONS_FLYING.keys())
        state = mock_base_state({
            "outcomes": outcomes,
            "trajectory_class": None,
        })
        
        battery = mock_battery_data(14, 5, 0.25)
        outcome = state.battery_monitor.on_battery(mock_message("battery", battery))
        self.assertEqual(outcome, "low_battery")
    
    def test_on_battery_message_via_messange_handlers(self):
        outcomes = list(STANDARD_TRANSITIONS_FLYING.keys())
        self.assertIn("low_battery", outcomes)
        state = mock_base_state({
            "outcomes": outcomes,
            "trajectory_class": None,
        })

        self.assertIsNotNone(state.battery_monitor)
        
        battery = mock_battery_data(14, 5, 0.99)
        self.assertEqual(battery.level, 0.99)
        outcome = state.handlers.notify(mock_message("battery", battery))
        self.assertIsNone(outcome)

        battery = mock_battery_data(14, 5, 0.33)
        outcome = state.handlers.notify(mock_message("battery", battery))
        self.assertIsNone(outcome)
        
        # The low battery threshold is 0.33...
        # so let's test that we get the "low_battery" outcome when the battery level is below 0.33
        battery = mock_battery_data(14, 5, 0.32999)
        self.assertEqual(battery.level, 0.32999)
        outcome = state.handlers.notify(mock_message("battery", battery))
        self.assertEqual(outcome, "low_battery")

        battery = mock_battery_data(14, 5, 0.25)
        outcome = state.handlers.notify(mock_message("battery", battery))
        self.assertEqual(outcome, "low_battery")

        battery = mock_battery_data(14, 5, 0.01)
        outcome = state.handlers.notify(mock_message("battery", battery))
        self.assertEqual(outcome, "low_battery")