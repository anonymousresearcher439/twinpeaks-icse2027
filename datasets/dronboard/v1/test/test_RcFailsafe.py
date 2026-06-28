"""Tests for the RcFailsafe component"""
import sys
import unittest

from unittest.mock import (
    Mock,
    patch
)

from mavros_msgs.msg import RCIn, State

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import RcFailsafe


class TestRcFailsafe(unittest.TestCase):
    @staticmethod
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def make_mock_BaseState(mock_heartbeat_status_handler):
        drone = Mock(spec=MAVROSDrone)
        message_senders = Mock(spec=ReusableMessageSenders)
        mqtt_client = Mock(spec=MQTTClient)
        kw = {'outcomes': ["human_control"]}
        return BaseState(
            drone=drone,
            reusable_message_senders=message_senders,
            mqtt_client=mqtt_client,
            **kw
        )

    @staticmethod
    def mock_message(message_type, data=None):
        return {
            'type': message_type,
            'data': data
        }
    
    @staticmethod
    def mock_state_msg(mode: str):
        mavros_msg =  Mock(spec=State, mode=mode)
        return TestRcFailsafe.mock_message('state', mavros_msg)
    
    @staticmethod
    def mock_rcin_msg(channel_values):
        mavros_msg = Mock(spec=RCIn, channels=channel_values)
        return TestRcFailsafe.mock_message('rcin', mavros_msg)

    def test_init_run_without_exception(self):
        state = TestRcFailsafe.make_mock_BaseState()
        self.assertIsNotNone(state._rc_failsafe_component)
        self.assertEqual(type(state._rc_failsafe_component), RcFailsafe)

    def test_position_mode_triggers_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        state.message_queue.put(TestRcFailsafe.mock_state_msg("POSCTL"))
        outcome = state.execute({})
        self.assertEqual("human_control", outcome)
    
    def test_that_auto_modes_do_not_trigger_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            TestRcFailsafe.mock_state_msg("AUTO.LAND"),
            TestRcFailsafe.mock_state_msg("AUTO.LOITER"),
            TestRcFailsafe.mock_state_msg("AUTO.TAKEOFF"),
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
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
            TestRcFailsafe.mock_state_msg("AUTO.RTL"),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "human_control")
    
    def test_that_altitude_control_mode_triggers_the_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
            TestRcFailsafe.mock_state_msg("ALTCTL"),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "human_control")
    
    def test_that_stabilized_mode_triggers_the_failsafe(self):
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
            TestRcFailsafe.mock_state_msg("STABILIZED"),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "human_control")
    
    def test_that_rcin_can_trigger_the_failsafe(self):
        rc_inputs = [1, 2, 3, 4, 1240, 6, 7, 8]
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        self.assertEqual(outcome, "human_control")
    
    def test_that_rcin_will_not_trigger_the_failsafe(self):
        rc_inputs = [1, 2, 3, 4, 5, 6, 7, 8]
        state = TestRcFailsafe.make_mock_BaseState()
        all_messages = [
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
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
            TestRcFailsafe.mock_state_msg("OFFBOARD"),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            TestRcFailsafe.mock_state_msg("AUTO.LAND"),
            TestRcFailsafe.mock_rcin_msg(rc_inputs),
            TestRcFailsafe.mock_state_msg("AUTO.TAKEOFF"),
            TestRcFailsafe.mock_message("shutdown")
        ]
        for msg in all_messages:
            state.message_queue.put(msg)
        
        outcome = state.execute({})
        # if rcin did't exit the state, then the shutdown message will
        # that's why we test for the error outcome 
        self.assertEqual(outcome, "error")


