import importlib
import json
import unittest
from unittest.mock import MagicMock, Mock, call, patch

from dr_onboard_autonomy.states.Sade import SadeEnter, SadeExit
from mock_types import mock_mqtt_client

from dr_onboard_autonomy.states.components import SadeCommunication
import mock_types

# get the module holding SadeCommunication
SadeCommunication_module = importlib.import_module(SadeCommunication.__module__)


def find_publish_call(publish_mock, topic):
    """Find the first publish call matching the topic in a mock's call_args_list.
    Returns the call tuple or None.
    """
    for c in publish_mock.call_args_list:
        # c[0] is args tuple, index 0 is the topic
        if c[0][0] == topic:
            return c
    return None

class TestSadeEnter(unittest.TestCase):
    def setUp(self):
        from .mock_types import mock_state_kwargs
        mqtt_cloud = mock_mqtt_client(
            uav_name="TEST DRONE",
            broker_address="127.0.0.1",
            local_ip="127.0.0.1",
        )
        self.kwargs = mock_state_kwargs({
            "drone_registration_id": "DRONE123",
            "pilot_id": "PILOT456",
            "owner_id": "OWNER789",
            "model_name": "MODEL_X",
            "mqtt_cloud": mqtt_cloud,
            "uav_id": "TESTING_UAV123"
        })
        del self.kwargs['trajectory_class']

    def test_init_sets_attributes(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        self.assertEqual(state.sade_zone_id, "ZONE1")
        self.assertEqual(state.drone_registration_id, "DRONE123")
        self.assertEqual(state.pilot_id, "PILOT456")

    @patch.object(SadeCommunication_module, "MQTTClient")
    def test_send_sade_messages(self, MQTTClient):
        ts = '2025-08-18T17:18:16.103+00:00'
        MQTTClient.timestamp.return_value = ts
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        
        state._is_sent = False
        state._send_sade_messages(userdata={})
        self.assertTrue(state._is_sent)
        state.sade_communication.send_uav_registration.assert_called_once()
        state.sade_communication.send_pilot_registration.assert_called_once()
        state.sade_communication.send_entry_request.assert_called_once()
        # Now we need to look at the messages we sent out of the mqtt_cloud client
        state.mqtt_cloud.publish.assert_called()

        drone_name = self.kwargs['uav_id']
        payload_register_model = {
            "droneID": self.kwargs['drone_registration_id'],
            "pilotID": self.kwargs['pilot_id'],
            "ownerID": self.kwargs['owner_id'],
            "modelName": self.kwargs['model_name'],
            "timestamp": ts,
        }
        payload_register = {
            "pilotID": self.kwargs['pilot_id'],
            "droneID": self.kwargs['drone_registration_id'],
            "ownerID": self.kwargs['owner_id'],
            "modelName": self.kwargs['model_name'],
            "timestamp": MQTTClient.timestamp(),
        }
        payload_request_access = {
            "pilotID": self.kwargs['pilot_id'],
            "droneID": self.kwargs['drone_registration_id'],
            "sade_zone_id": "ZONE1",
            "uavID": self.kwargs['uav_id'],
            "response_topic": f"drone/{drone_name}/sade_entry_response",
            "timestamp": MQTTClient.timestamp(),
        }

        calls = [
            call("sade_manager/uav/register_model", payload_register_model, qos=1),
            call("sade_manager/uav/register", payload_register, qos=1),
            call("sade_manager/sade/request_access", payload_request_access, qos=1)
        ]
        state.mqtt_cloud.publish.assert_has_calls(calls, any_order=True)

    def test_on_timeout(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        self.assertEqual(state.on_timeout(userdata={}), "timeout")

    def test_receive_entry_response1(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state.status_message = Mock(wraps=state.status_message)
        state._send_sade_messages(userdata={})
        # now we find the publish call with the topic = "sade_manager/sade/request_access"
        # really we need to look at the payload and take values from it
        state.mqtt_cloud.publish.assert_called()
        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/request_access")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']
        decision = 'ALLOW'

        response = {
            'sade_zone_id': zone,
            'decision': decision,
            'fields': [
                "gps_status",
                "vibration",
                "rc_out",
                "ekf_status",
                "target_attitude",
            ]
        }
        # need to make a Mock MQTT message
        msg = mock_types.mock_mqtt_message(
            "sade_entry_response",
            response_topic,
            json.dumps(response).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_entry_response(msg)
        self.assertEqual(outcome, 'allow')
        # Make sure we updated the status message fields
        state.status_message.update_status_message.assert_called_once_with([
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
            "target_attitude",
        ])

    def test_receive_entry_response_deny(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state.status_message = Mock(wraps=state.status_message)
        state._send_sade_messages(userdata={})
        # find the publish call for request_access to extract payload
        state.mqtt_cloud.publish.assert_called()

        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/request_access")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']

        response = {
            'sade_zone_id': zone,
            'decision': 'DENY',
            'fields': [
                "gps_status",
                "vibration",
                "rc_out",
                "ekf_status",
                "target_attitude",
            ]
        }
        msg = mock_types.mock_mqtt_message(
            "sade_entry_response",
            response_topic,
            json.dumps(response).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_entry_response(msg)
        self.assertEqual(outcome, 'deny')
        state.status_message.update_status_message.assert_called_once_with([
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
            "target_attitude",
        ])

    def test_receive_entry_response_deny_no_fields(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state.status_message = Mock(wraps=state.status_message)
        state._send_sade_messages(userdata={})
        # find the publish call for request_access to extract payload
        state.mqtt_cloud.publish.assert_called()

        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/request_access")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']

        # Build response without optional 'fields'
        response = {
            'sade_zone_id': zone,
            'decision': 'DENY',
        }
        msg = mock_types.mock_mqtt_message(
            "sade_entry_response",
            response_topic,
            json.dumps(response).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_entry_response(msg)
        self.assertEqual(outcome, 'deny')
        # Ensure no status field update attempted when 'fields' is missing
        state.status_message.update_status_message.assert_not_called()

    def test_receive_entry_response_proving_ground_no_fields(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state.status_message = Mock(wraps=state.status_message)
        state._send_sade_messages(userdata={})
        # find the publish call for request_access to extract payload
        state.mqtt_cloud.publish.assert_called()

        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/request_access")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']

        # Build response without optional 'fields'
        response = {
            'sade_zone_id': zone,
            'decision': 'PROVING_GROUND',
        }
        msg = mock_types.mock_mqtt_message(
            "sade_entry_response",
            response_topic,
            json.dumps(response).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_entry_response(msg)
        self.assertEqual(outcome, 'proving_ground')
        # Ensure no status field update attempted when 'fields' is missing
        state.status_message.update_status_message.assert_not_called()

    def test_receive_entry_response_proving_ground_with_fields(self):
        state = SadeEnter(sade_zone_id="ZONE1", timeout=10.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state.status_message = Mock(wraps=state.status_message)
        state._send_sade_messages(userdata={})
        # find the publish call for request_access to extract payload
        state.mqtt_cloud.publish.assert_called()

        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/request_access")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']

        fields = [
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
            "target_attitude",
        ]
        response = {
            'sade_zone_id': zone,
            'decision': 'PROVING_GROUND',
            'fields': fields,
        }
        msg = mock_types.mock_mqtt_message(
            "sade_entry_response",
            response_topic,
            json.dumps(response).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_entry_response(msg)
        self.assertEqual(outcome, 'proving_ground')
        state.status_message.update_status_message.assert_called_once_with(fields)


class TestSadeExit(unittest.TestCase):
    def setUp(self):
        from .mock_types import mock_state_kwargs
        mqtt_cloud = mock_mqtt_client(
            uav_name="TEST DRONE",
            broker_address="127.0.0.1",
            local_ip="127.0.0.1",
        )
        self.kwargs = mock_state_kwargs({
            "drone_registration_id": "DRONE123",
            "pilot_id": "PILOT456",
            "owner_id": "OWNER789",
            "model_name": "MODEL_X",
            "mqtt_cloud": mqtt_cloud,
            "uav_id": "TESTING_UAV123"
        })
        del self.kwargs['trajectory_class']
        

    def test_init_sets_attributes(self):
        state = SadeExit(sade_zone_id="ZONE2", timeout=5.0, **self.kwargs)
        self.assertEqual(state.sade_zone_id, "ZONE2")
        self.assertEqual(state.drone_registration_id, "DRONE123")
        self.assertEqual(state.pilot_id, "PILOT456")

    @patch.object(SadeCommunication_module, "MQTTClient")
    def test_send_sade_exit(self, MQTTClient):
        ts = '2025-08-18T17:18:16.103+00:00'
        MQTTClient.timestamp.return_value = ts

        state = SadeExit(sade_zone_id="ZONE2", timeout=5.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        state._is_sent = False
        state._send_sade_messages(userdata={})
        self.assertTrue(state._is_sent)
        state.sade_communication.send_sade_exit.assert_called_once()

        # Validate MQTT publish payload and topic
        drone_name = self.kwargs.get('uav_id', 'test_drone')
        payload_exit = {
            "pilotID": self.kwargs['pilot_id'],
            "droneID": self.kwargs['drone_registration_id'],
            "sade_zone_id": "ZONE2",
            "response_topic": f"drone/{drone_name}/sade_exit_response",
            "timestamp": MQTTClient.timestamp(),
        }
        calls = [
            call("sade_manager/sade/exit", payload_exit, qos=1),
        ]
        state.mqtt_cloud.publish.assert_has_calls(calls, any_order=True)

    def test_on_timeout(self):
        state = SadeExit(sade_zone_id="ZONE2", timeout=5.0, **self.kwargs)
        self.assertEqual(state.on_timeout(userdata={}), "timeout")
    
    def test_receive_exit_confirmation(self):
        state = SadeExit(sade_zone_id="ZONE2", timeout=5.0, **self.kwargs)
        state.sade_communication = Mock(wraps=state.sade_communication)
        # we need to send the exit message so that we can inspect the payload
        # and grab the response topic
        state._send_sade_messages(userdata={})
        state.mqtt_cloud.publish.assert_called()
        entry_call = find_publish_call(state.mqtt_cloud.publish, "sade_manager/sade/exit")
        assert entry_call
        payload = entry_call[0][1]
        assert payload
        response_topic = payload['response_topic']
        zone = payload['sade_zone_id']

        # make an MQTT message
        msg = mock_types.mock_mqtt_message(
            "sade_exit_confirmation",
            response_topic,
            json.dumps({"sade_zone_id": zone, "status": "confirmed"}).encode('utf-8'),
        )
        outcome = state.sade_communication._on_sade_exit_confirmation(msg)
        self.assertEqual(outcome, 'success_sade_exit')

if __name__ == "__main__":
    unittest.main()