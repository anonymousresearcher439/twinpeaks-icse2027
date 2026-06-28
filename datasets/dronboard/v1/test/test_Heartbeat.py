import json
import unittest

from unittest.mock import NonCallableMagicMock, Mock, PropertyMock
from dr_onboard_autonomy.mavlink import HeartbeatSender

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.HeartbeatHover import HeartbeatHover


def mock_drone():
        drone = NonCallableMagicMock(spec=MAVROSDrone)
        drone.onboard_heartbeat = NonCallableMagicMock(spec=HeartbeatSender)
        return drone


class TestHeartbeatStatusHandler(unittest.TestCase):
    def setUp(self):
        drone = mock_drone()
        mock_msg_sender = NonCallableMagicMock(spec=AbstractMessageSender)
        reusable_message_senders = NonCallableMagicMock(spec=ReusableMessageSenders)
        reusable_message_senders.configure_mock(
            **{"find.return_value": mock_msg_sender}
        )
        local_mqtt_client = NonCallableMagicMock(spec=MQTTClient)
        
        self.args = {
            "drone": drone,
            "reusable_message_senders": reusable_message_senders,
            "uav_id": "uav_id",
            "local_mqtt_client": local_mqtt_client
        }


    def test_on_heartbeat_status_message_hover(self):
        from src.dr_onboard_autonomy.states.components.Heartbeat import HeartbeatStatusHandler
        
        mock_base_state = NonCallableMagicMock()
        mock_base_state.args = self.args

        
        
        mock_mqtt_payload_sequence = [
            {
                "type": "heartbeat_status",
                "data": {"status" : "continue"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "hover"}
            }
        ]

        heartbeat_status_handler = HeartbeatStatusHandler(mock_base_state)
        heartbeat_status_handler.heartbeat_hover = NonCallableMagicMock(spec=HeartbeatHover)
        heartbeat_status_handler.heartbeat_hover.execute.return_value = "succeeded_hover"

        for mock_payload in mock_mqtt_payload_sequence:
            hover_outcome = heartbeat_status_handler._on_heartbeat_status_message(
                message=mock_payload
            )

        self.assertIsNone(hover_outcome)
        mock_base_state.drone.update_data.assert_called_with("heartbeat_status", "hover")
        mock_base_state.stop_message_senders.assert_called_once_with()
        mock_base_state.trajectory.pause.assert_called_once_with()
        mock_base_state.message_queue.queue.clear.assert_called_once_with()
        '''
        expected calls with HearbeatHover returning 'succeeded_hover'
        '''
        mock_base_state.start_message_senders.assert_called_once_with()
        mock_base_state.trajectory.resume.assert_called_once_with()


    def test_on_heartbeat_status_hover_failed_outcome(self):
        from src.dr_onboard_autonomy.states.components.Heartbeat import HeartbeatStatusHandler
        
        mock_base_state = NonCallableMagicMock()
        mock_base_state.args = self.args
        failure_outcome = "some failure outcome"

        mock_mqtt_payload_sequence = [
            {
                "type": "heartbeat_status",
                "data": {"status" : "continue"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "hover"}
            }
        ]

        heartbeat_status_handler = HeartbeatStatusHandler(mock_base_state)
        heartbeat_status_handler.heartbeat_hover = NonCallableMagicMock(spec=HeartbeatHover)
        heartbeat_status_handler.heartbeat_hover.execute.return_value = failure_outcome

        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            hover_outcome = heartbeat_status_handler._on_heartbeat_status_message(
                message=mock_mqtt_payload
            )

        self.assertEqual(hover_outcome, failure_outcome)


    def test_on_heartbeat_no_trajectory_instance(self):
        from src.dr_onboard_autonomy.states.components.Heartbeat import HeartbeatStatusHandler
        
        mock_base_state = NonCallableMagicMock()
        mock_base_state.args = self.args

        mock_mqtt_payload_sequence = [
            {
                "type": "heartbeat_status",
                "data": {"status" : "continue"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "hover"}
            }
        ]

        heartbeat_status_handler = HeartbeatStatusHandler(mock_base_state)
        heartbeat_status_handler._state_trajectory = None

        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            outcome = heartbeat_status_handler._on_heartbeat_status_message(
                message=mock_mqtt_payload
            )

            self.assertIsNone(outcome)


    def test_on_heartbeat_status_message_continue(self):
        from src.dr_onboard_autonomy.states.components.Heartbeat import HeartbeatStatusHandler
        
        mock_base_state = NonCallableMagicMock()
        mock_base_state.args = self.args

        mock_mqtt_payload_sequence = [
            {
                "type": "heartbeat_status",
                "data": {"status" : "hover"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "continue"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "continue"}
            },
        ]

        heartbeat_status_handler = HeartbeatStatusHandler(mock_base_state)
        heartbeat_status_handler.heartbeat_hover = NonCallableMagicMock(spec=HeartbeatHover)
        heartbeat_status_handler.heartbeat_hover.execute.return_value = "succeeded_hover"

        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            continue_outcome = heartbeat_status_handler._on_heartbeat_status_message(
                message=mock_mqtt_payload
            )

        self.assertIsNone(continue_outcome)
        mock_base_state.drone.update_data.assert_called_with("heartbeat_status", "continue")


    def test_on_heartbeat_status_message_rtl(self):
        from src.dr_onboard_autonomy.states.components.Heartbeat import HeartbeatStatusHandler
        
        mock_base_state = NonCallableMagicMock()
        mock_base_state.args = self.args

        mock_mqtt_payload_sequence = [
            {
                "type": "heartbeat_status",
                "data": {"status" : "hover"}
            },
            {
                "type": "heartbeat_status",
                "data": {"status" : "rtl"}
            }
        ]

        heartbeat_status_handler = HeartbeatStatusHandler(mock_base_state)
        heartbeat_status_handler.heartbeat_hover = NonCallableMagicMock(spec=HeartbeatHover)
        heartbeat_status_handler.heartbeat_hover.execute.return_value = "succeeded_hover"
        heartbeat_status_handler._state_kwargs = {}
        heartbeat_status_handler._heartbeat_hover_time = 1000

        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            rtl_outcome = heartbeat_status_handler._on_heartbeat_status_message(
                message=mock_mqtt_payload
            )

        self.assertEqual(rtl_outcome, "rtl")


class TestHeartbeatHover(unittest.TestCase):
    def setUp(self):
        self.mock_mqtt_client = NonCallableMagicMock(spec=MQTTClient)
        self.mock_reusable_message_senders = NonCallableMagicMock(spec=ReusableMessageSenders)
        self.mock_userdata = NonCallableMagicMock()


    def test_on_entry(self):
        from src.dr_onboard_autonomy.states.HeartbeatHover import HeartbeatHover
        
        drone = mock_drone()
        drone.onboard_heartbeat = NonCallableMagicMock(spec=HeartbeatSender)
        key_args = {
            "drone" : drone,
            "mqtt_client" : self.mock_mqtt_client,
            "reusable_message_senders" : self.mock_reusable_message_senders
        }

        heartbeat_hover = HeartbeatHover(**key_args)
        self.assertNotIn("heartbeat_status_handler", dir(heartbeat_hover))

        entry_outcome = heartbeat_hover.on_entry(userdata=None)

        self.assertIsNone(entry_outcome)
        drone.hover.assert_called_once_with()

    
    def test_on_entry_hover_error(self):
        from src.dr_onboard_autonomy.states.HeartbeatHover import HeartbeatHover
        
        drone = mock_drone()
        drone.hover.return_value = False
        key_args = {
            "drone" : drone,
            "mqtt_client" : self.mock_mqtt_client,
            "reusable_message_senders" : self.mock_reusable_message_senders
        }

        heartbeat_hover = HeartbeatHover(**key_args)

        entry_outcome = heartbeat_hover.on_entry(userdata=None)
    
        self.assertEqual(entry_outcome, "error")


    def test_on_heartbeat_status_message(self):
        from src.dr_onboard_autonomy.states.HeartbeatHover import HeartbeatHover
        
        key_args = {
            "drone": mock_drone(),
            "mqtt_client" : self.mock_mqtt_client,
            "reusable_message_senders" : self.mock_reusable_message_senders
        }

        heartbeat_hover = HeartbeatHover(**key_args)

        continue_status_message = {
            "type": "heartbeat_status",
            "data": {
                "status": "continue"
            }
        }


        message_outcome = heartbeat_hover._on_heartbeat_status_message(continue_status_message)
        self.assertEqual(message_outcome, "succeeded_hover")
        
        message_outcome = heartbeat_hover.handlers.notify(continue_status_message)
        self.assertEqual(message_outcome, "succeeded_hover")
        

        continue_status_message = {
            "type": "heartbeat_status",
            "data": {
                "status": "rtl"
            }
        }

        message_outcome = heartbeat_hover._on_heartbeat_status_message(continue_status_message)

        self.assertEqual(message_outcome, "rtl")