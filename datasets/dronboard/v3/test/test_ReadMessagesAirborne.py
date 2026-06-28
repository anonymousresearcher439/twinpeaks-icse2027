import unittest

from queue import Queue

from unittest.mock import NonCallableMock

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.models import FCUCopterMode
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import ReadMessagesAirborne

from .mock_types import (
    mock_copter_drone,
    mock_message,
    mock_message_sender_position_message,
    mock_message_sender_state_message,
    mock_shutdown_message,
)


class TestBaseState(unittest.TestCase):
    def setUp(self):
        self.mock_air_lease_service = NonCallableMock(spec=AirLeaseService)
        self.mock_drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.mock_reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.mock_reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)

        self.base_state_kwargs = {
            "air_lease_service": self.mock_air_lease_service,
            "drone": self.mock_drone,
            "reusable_message_senders": self.mock_reusable_msg_senders,
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "outcomes": [],
            "data": {}
        }


    def test_init(self):
        q = Queue()
        required_message_names = ["position", "mock_example"]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )

        # make sure the ReadMessagesAirborne initalizer set all the required fields
        actual_fields = {
            "air_lease_service": lambda: state.air_lease_service,
            "reusable_message_senders": lambda: state.reusable_message_senders,
            "local_mqtt_client": lambda: state.local_mqtt_client,
            "mqtt_client": lambda: state.mqtt_client,
            "name": lambda: state.name,
            "drone": lambda: state.drone,
        }
        expected_fields = {
            "air_lease_service":lambda: self.mock_air_lease_service,
            "reusable_message_senders":lambda: self.mock_reusable_msg_senders,
            "local_mqtt_client":lambda: self.mock_mqtt_local,
            "mqtt_client":lambda: self.mock_mqtt,
            "name":lambda: "ReadMessagesAirborne",
            "drone":lambda: self.mock_drone,
        }
        
        for field_name in [
                "air_lease_service",
                "reusable_message_senders",
                "local_mqtt_client",
                "mqtt_client",
                "name",
                "drone",
        ]:
            get_actual_field = actual_fields[field_name]
            actual = get_actual_field()

            get_expected_field = expected_fields[field_name]
            expected = get_expected_field()

            self.assertEqual(
                actual,
                expected,
                f"ReadMessagesAirborne.{field_name} doesn't have the expected value."
            )

        # make sure ReadMessagesAirborne registers a message handler for
        # each type of message in message_names
        active_message_handlers = state.handlers.active_handlers.keys()
        for msg_name in required_message_names:
            assert msg_name in active_message_handlers

    def test_execute_with_one_required_message(self):
        q = Queue()
        required_message_names = ["mock_example"]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )

        state.message_queue.put({
            'type': "mock_example",
            'data': "hello"
        })

        state.message_queue.put({
            'type': "shutdown",
            'data': "hello"
        })

        outcome = state.execute(None)
        self.assertEqual(outcome, "succeeded", "ReadMessagesAirborne exited with the wrong outcome")
    
    def test_execute_with_3_required_message(self):
        q = Queue()
        required_message_names = ["mock_example", "mock_example2", "mock_example3"]
        required_messages = [
            {
                'type': "mock_example",
                'data': "hello"
            },
            {
                'type': "mock_example2",
                'data': "hello"
            },
            {
                'type': "mock_example3",
                'data': "hello"
            },
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in required_messages:
            state.message_queue.put(msg)

        state.message_queue.put({
            'type': "shutdown",
            'data': "hello"
        })

        outcome = state.execute(None)
        self.assertEqual(outcome, "succeeded", "ReadMessagesAirborne exited with the wrong outcome")
    
    def test_execute_will_shutdown(self):
        q = Queue()
        required_message_names = ["mock_example", "mock_example2", "mock_example3"]
        input_messages = [
            {
                'type': "mock_example",
                'data': "hello"
            },
            {
                'type': "mock_example2",
                'data': "hello"
            },
            {
                'type': "shutdown",
                'data': 0
            },
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "error", "ReadMessagesAirborne exited with the wrong outcome")
    
    def test_execute_will_return_the_newest_message_of_a_given_type(self):
        q = Queue()
        required_message_names = ["mock_example", "mock_example2"]
        input_messages = [
            {
                'type': "mock_example",
                'data': 0
            },
            {
                'type': "mock_example",
                'data': 1
            },
            {
                'type': "mock_example2",
                'data': 0
            },
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "succeeded", "ReadMessagesAirborne exited with the wrong outcome")

        messages = q.get()
        self.assertEqual(messages["mock_example"], 1, "ReadMessagesAirborne returned the wrong message")
        self.assertEqual(messages["mock_example2"], 0, "ReadMessagesAirborne returned the wrong message")


    def test_human_control_failsafe_stabilized(self):
        q = Queue()
        required_message_names = ["mock_example"]
        input_messages = [
            mock_message_sender_position_message(0, 1, 2),
            mock_message_sender_state_message(),
            mock_message_sender_state_message(mode=FCUCopterMode.STABILIZE),
            mock_shutdown_message(),
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "failsafe", "ReadMessagesAirborne exited with the wrong outcome")


    def test_human_control_failsafe_altitude_control(self):
        q = Queue()
        required_message_names = ["mock_example"]
        input_messages = [
            mock_message_sender_position_message(0, 1, 2),
            mock_message_sender_state_message(),
            mock_message_sender_state_message(mode=FCUCopterMode.ALT_HOLD),
            mock_shutdown_message(),
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "failsafe", "ReadMessagesAirborne exited with the wrong outcome")


    def test_human_control_failsafe_position_control(self):
        q = Queue()
        required_message_names = ["mock_example"]
        input_messages = [
            mock_message_sender_position_message(0, 1, 2),
            mock_message_sender_state_message(),
            mock_message_sender_state_message(mode=FCUCopterMode.POS_HOLD),
            mock_shutdown_message(),
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "failsafe", "ReadMessagesAirborne exited with the wrong outcome")

    def test_RTL_failsafe(self):
        q = Queue()
        required_message_names = ["mock_example"]
        input_messages = [
            mock_message_sender_position_message(0, 1, 2),
            mock_message_sender_state_message(),
            mock_message_sender_state_message(mode=FCUCopterMode.RTL),
            mock_shutdown_message(),
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "failsafe", "ReadMessagesAirborne exited with the wrong outcome")


    def test_abort_failsafe(self):
        q = Queue()
        required_message_names = ["mock_example"]
        input_messages = [
            mock_message_sender_position_message(0, 1, 2),
            mock_message_sender_state_message(),
            mock_message("abort", 0),
            mock_shutdown_message(),
        ]
        state = ReadMessagesAirborne(
            message_names=required_message_names,
            return_channel=q,
            **self.base_state_kwargs
        )
        for msg in input_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "abort", "ReadMessagesAirborne exited with the wrong outcome")