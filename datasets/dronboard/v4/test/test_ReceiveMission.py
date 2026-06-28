import importlib
import json
import queue
import sys
import time
import unittest
import inspect

from unittest.mock import call, patch, NonCallableMock, Mock

from dr_onboard_autonomy.states import ReceiveMission

from . import mock_types


ReceiveMission_module = importlib.import_module(ReceiveMission.__module__)

class TestReceiverMission(unittest.TestCase):
    def test_init(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        with patch.object(ReceiveMission_module, "RepeatTimer") as mockRepeatTimer:
            state = ReceiveMission(**kwargs)
            # make sure we called it twice with these args
            required_calls=[
                call("log_info", 2.5),
                call("new_drone_message", 0.5)
            ]
            mockRepeatTimer.assert_has_calls(required_calls, any_order=True)

        self.assertIsInstance(state, ReceiveMission)
    
    def test_new_drone_message_is_sent(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        state = ReceiveMission(**kwargs)
        with patch.object(state.mqtt_client, "publish") as mock_publish:
            mock_msg = mock_types.mock_message("new_drone_message", 0.5)
            state.handlers.notify(mock_msg)
            # mock_publish.assert_called_once()
            args, kwargs = mock_publish.call_args
            topic = args[0]
            msg = args[1]
            self.assertEqual(topic, "new_drone")
            self.assertIn("camera", msg)
    
    def test_new_drone_message_is_sent_every_fourth_event(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        state = ReceiveMission(**kwargs)
        with patch.object(state.mqtt_client, "publish") as mock_publish:
            mock_msg = mock_types.mock_message("new_drone_message", 0.5)
            mock_publish.assert_not_called()
            for i in range(16):
                state.handlers.notify(mock_msg)
                if i in [0, 4, 8, 12]:
                    mock_publish.assert_called()
                mock_publish.reset_mock()
            
    def test_new_drone_message_is_sent_when_camera_not_provided(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        del kwargs["camera_config"]
        state = ReceiveMission(**kwargs)
        with patch.object(state.mqtt_client, "publish") as mock_publish:
            mock_msg = mock_types.mock_message("new_drone_message", 0.5)
            state.handlers.notify(mock_msg)
            # mock_publish.assert_called_once()
            args, kwargs = mock_publish.call_args
            topic = args[0]
            msg = args[1]
            self.assertEqual(topic, "new_drone")
            self.assertNotIn("camera", msg)
        