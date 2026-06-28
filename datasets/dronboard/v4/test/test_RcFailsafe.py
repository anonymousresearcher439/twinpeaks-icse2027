"""Tests for the RcFailsafe component"""
import sys
import unittest

from unittest.mock import (
    Mock,
    patch
)

from mavros_msgs.msg import RCIn

from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.models import FCUCopterMode
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import RcFailsafe
from .mock_types import mock_copter_drone, mock_message_sender_state_message
import mock_types


class TestRcFailsafe(unittest.TestCase):
    @staticmethod
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def make_mock_BaseState(mock_heartbeat_status_handler):
        drone = mock_copter_drone()
        message_senders = Mock(spec=ReusableMessageSenders)
        mqtt_client = Mock(spec=MQTTClient)
        
        args = dict(
            drone=drone,
            reusable_message_senders=message_senders,
            mqtt_client=mqtt_client,
            outcomes=["failsafe"],
            trajectory_class=None,
        )
        return mock_types.mock_base_state(args)

    @staticmethod
    def mock_message(message_type, data=None):
        return {
            'type': message_type,
            'data': data
        }

    
    @staticmethod
    def mock_rcin_msg(channel_values):
        mavros_msg = Mock(spec=RCIn, channels=channel_values)
        return TestRcFailsafe.mock_message('rcin', mavros_msg)

    def test_init_run_without_exception(self):
        state = TestRcFailsafe.make_mock_BaseState()
        self.assertIsNotNone(state.rc_failsafe)
        self.assertEqual(type(state.rc_failsafe), RcFailsafe)

    def test_position_mode_triggers_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        state.message_queue.put(mock_message_sender_state_message(mode=FCUCopterMode.POS_HOLD))
        outcome = state.execute({})
        self.assertEqual("failsafe", outcome)
    
    def test_that_auto_modes_do_not_trigger_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.LOITER),
            mock_message_sender_state_message(mode=FCUCopterMode.TAKEOFF),
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        # expect the error outcome if the state exited because of the shutdown message
        self.assertEqual(outcome, "error")
    
    def test_that_auto_modes_do_not_trigger_failsafe_when_we_remove_a_mode(self):
        """We need to make sure that the failsafe does not trigger when we remove a mode from the failsafe set.
        """
        state = TestRcFailsafe.make_mock_BaseState()
        state.rc_failsafe.remove_mode(FCUCopterMode.LAND)
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.LAND),
            mock_message_sender_state_message(mode=FCUCopterMode.LOITER),
            mock_message_sender_state_message(mode=FCUCopterMode.TAKEOFF),
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        # expect the error outcome if the state exited because of the shutdown message
        self.assertEqual(outcome, "error")
    
    def test_that_RTL_triggers_the_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.RTL),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "failsafe")
    
    def test_that_altitude_control_mode_triggers_the_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            mock_message_sender_state_message(mode=FCUCopterMode.ALT_HOLD),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "failsafe")
    
    def test_that_stabilized_mode_triggers_the_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            mock_message_sender_state_message(mode=FCUCopterMode.STABILIZE),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "failsafe")
    
    def test_that_rcin_will_not_trigger_the_failsafe(self):
        rc_inputs = [1, 2, 3, 4, 5, 6, 7, 8]
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        # if rcin did't exit the state, then the shutdown message will
        # that's why we test for the error outcome 
        self.assertEqual(outcome, "error")
    
    def test_that_rcin_and_state_will_not_trigger_the_failsafe(self):
        rc_inputs = [1, 2, 3, 4, 5, 6, 7, 8]
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            mock_message_sender_state_message(mode=FCUCopterMode.LOITER),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            mock_message_sender_state_message(mode=FCUCopterMode.TAKEOFF),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        # if rcin did't exit the state, then the shutdown message will
        # that's why we test for the error outcome
        self.assertEqual(outcome, "error")


