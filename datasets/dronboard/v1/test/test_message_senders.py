#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_message_senders2"
from genpy import message
import roslib

# roslib.load_manifest(PKG)
import json
import unittest
from unittest.mock import (
    MagicMock,
    Mock,
    NonCallableMock,
    patch,
    PropertyMock
)
from threading import Event
from queue import Empty, Queue

from dr_onboard_autonomy.message_senders import RepeatTimer, ReliableMessageSender


class TestRepeatTimer(unittest.TestCase):
    def test_RepeatTimer_init(self):
        timer = RepeatTimer("test_timer", 0.25)
        self.assertEqual(timer.is_running(), False)

    def test_RepeatTimer_start_and_stop(self):
        timer = RepeatTimer("test_timer", 0.25)
        q = Queue()
        timer.start(q.put)
        m = q.get(timeout=0.5)

        self.assertEqual(m["type"], "test_timer")
        self.assertTrue(m["data"] >= 0.25)

        timer.stop()
        with self.assertRaises(Empty):
            # We should not get a timer message after we stopped the message_sender
            m = q.get(timeout=0.5)

    def test_two_timers_at_once(self):
        timer1 = RepeatTimer("timer1", 0.25)
        timer2 = RepeatTimer("timer2", 0.35)

        q = Queue()
        timer1.start(q.put)
        timer2.start(q.put)

        m1 = q.get(timeout=0.3)
        m2 = q.get(timeout=0.3)

        timer1.stop()
        timer2.stop()

        self.assertEqual(m1["type"], "timer1")
        self.assertEqual(m2["type"], "timer2")
        self.assertTrue(m1["data"] >= 0.25)
        self.assertTrue(m2["data"] >= 0.35)


class TestReliableMessageSender(unittest.TestCase):
    class MockMessageSender:
        def __init__(self):
            self.start_event = Event()
            self.callback = None
            self.name = "MockMessageSender"

        def start(self, callback):
            self.callback = callback
            self.start_event.set()

        def stop(self):
            self.start_event.clear()
            self.callback = None

        def send(self, message_type, payload):
            message = {"type": message_type, "data": payload}
            self.callback(message)

    def test_ReliableMessage_starts_and_stops_the_underlying_message_sender(self):
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            mock_message_sender.start_event.wait(timeout=5.0)
            mock_message_sender.start.assert_called_once()
            mock_message_sender.stop.assert_not_called()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

        mock_message_sender.start.assert_called_once()
        mock_message_sender.stop.assert_called_once()

    def test_ReliableMessage_buffers_1_message_and_delivers_it_later(self):
        """In this test we start a reliable message sender, and send it a message while it's not
        routing messages to a state machine. Then we test that it delivers the message later, when
        we call start()
        """
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)
        reliable_message_sender.start_loop()
        try:
            mock_message_sender.send("test", "data")
            reliable_message_sender.start(state_machine_queue.put)
            message1 = state_machine_queue.get(timeout=5.0)

            self.assertEqual(message1["type"], "test")
            self.assertEqual(message1["data"], "data")
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_buffers_many_messages_and_delivers_them_later(self):
        """In this test we start a reliable message sender, and send it many messages while it's not
        routing messages to a state machine. Then we test that it delivers all the messages later,
        after we call start()
        """
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        message_data = [("test", "0"), ("test", "1"), ("test", "2")]

        reliable_message_sender.start_loop()
        try:
            for input_data in message_data:
                mock_message_sender.send(*input_data)

            reliable_message_sender.start(state_machine_queue.put)

            for expected_data in message_data:
                message = state_machine_queue.get(timeout=5.0)
                message_type, message_payload = expected_data
                self.assertEqual(message["type"], message_type)
                self.assertEqual(message["data"], message_payload)
                done_func = message["done"]
                done_func()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_delivers_messages_immediately(self):
        """In this test we start a reliable message sender, and we tell it to routing messages to a state machine before we give it messages.
        Then we test that it delivers all the messages when there is no need to buffer them.
        """
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        message_data = [("test", "0"), ("test", "1"), ("test", "2")]

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)

            for original_message in message_data:
                original_message_type, original_message_payload = original_message
                mock_message_sender.send(
                    original_message_type, original_message_payload
                )
                out_message = state_machine_queue.get(timeout=5.0)

                self.assertEqual(out_message["type"], original_message_type)
                self.assertEqual(out_message["data"], original_message_payload)
                self.assertTrue(state_machine_queue.empty())

                done_func = out_message["done"]
                done_func()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_delivers_messages_again(self):
        """In this test, we make sure a message gets delivered more than once. This should happen
        when an earlier delivery is not acknowledged. This test simulates what would happen if a
        state transitioned before receiving the message.
        """
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)
            original_message_type, original_message_payload = "type", "data"
            mock_message_sender.send(original_message_type, original_message_payload)
            out_message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message1["type"], original_message_type)
            self.assertEqual(out_message1["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message2 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message2["type"], original_message_type)
            self.assertEqual(out_message2["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_delivers_messages_twice_then_moves_on_when_message2_arrives_after_ack1_and_after_stop2(
        self,
    ):
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)
            original_message_type, original_message_payload = "type", "data"
            mock_message_sender.send(original_message_type, original_message_payload)
            out_message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message1["type"], original_message_type)
            self.assertEqual(out_message1["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()  # stop1

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message2 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message2["type"], original_message_type)
            self.assertEqual(out_message2["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())
            ack1_func = out_message2["done"]
            ack1_func()

            reliable_message_sender.stop()  # stop2

            message2_type, message2_data = "test2", "payload2"
            mock_message_sender.send(message2_type, message2_data)

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message3 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message3["type"], message2_type)
            self.assertEqual(out_message3["data"], message2_data)
            ack2_func = out_message2["done"]
            ack2_func()

        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_delivers_messages_twice_then_moves_on_when_message2_arrives_before_ack1_and_before_stop2(
        self,
    ):
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)
            original_message_type, original_message_payload = "type", "data"
            mock_message_sender.send(original_message_type, original_message_payload)
            out_message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message1["type"], original_message_type)
            self.assertEqual(out_message1["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.stop()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message2 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message2["type"], original_message_type)
            self.assertEqual(out_message2["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())

            message2_type, message2_data = "test2", "payload2"
            mock_message_sender.send(message2_type, message2_data)

            ack1_func = out_message2["done"]
            ack1_func()

            reliable_message_sender.stop()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message3 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message3["type"], message2_type)
            self.assertEqual(out_message3["data"], message2_data)
            ack2_func = out_message2["done"]
            ack2_func()

        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_delivers_messages_twice_then_moves_on_when_message2_arrives_after_ack1_and_before_stop2(
        self,
    ):
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)

            original_message_type, original_message_payload = "type", "data"
            mock_message_sender.send(original_message_type, original_message_payload)

            out_message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message1["type"], original_message_type)
            self.assertEqual(out_message1["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())

            reliable_message_sender.stop()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message2 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message2["type"], original_message_type)
            self.assertEqual(out_message2["data"], original_message_payload)
            self.assertTrue(state_machine_queue.empty())

            ack1_func = out_message2["done"]
            ack1_func()

            message2_type, message2_data = "test2", "payload2"
            mock_message_sender.send(message2_type, message2_data)

            reliable_message_sender.stop()

            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message3 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message3["type"], message2_type)
            self.assertEqual(out_message3["data"], message2_data)
            ack2_func = out_message2["done"]
            ack2_func()

        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_when_we_call_methods_in_the_following_order_start_stop_send_start_stop(
        self,
    ):
        """In this test, we call start, stop, send, start, and stop"""
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.start(state_machine_queue.put)
            reliable_message_sender.stop()
            self.assertTrue(state_machine_queue.empty())

            message1_type, message1_payload = "test1", "data1"
            mock_message_sender.send(message1_type, message1_payload)
            self.assertTrue(state_machine_queue.empty())

            reliable_message_sender.start(state_machine_queue.put)
            out_message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message1["type"], message1_type)
            self.assertEqual(out_message1["data"], message1_payload)
            self.assertTrue(state_machine_queue.empty())
            ack1_func = out_message1["done"]
            ack1_func()
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_when_we_change_the_message_before_ack(self):
        """In this test, we will change the message before we call ack to make sure we have a
        copy that we own.
        """
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        reliable_message_sender.start_loop()
        try:
            message1_type, message1_payload = "1", "1"
            mock_message_sender.send(message1_type, message1_payload)
            self.assertTrue(state_machine_queue.empty())
            reliable_message_sender.start(state_machine_queue.put)
            out_message1 = state_machine_queue.get(timeout=5.0)
            ack1_func = out_message1["done"]
            for key in out_message1:
                out_message1[key] = None
            ack1_func()

            message2_type, message2_payload = "2", "2"
            mock_message_sender.send(message2_type, message2_payload)
            out_message2 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(out_message2["type"], message2_type)
            self.assertEqual(out_message2["data"], message2_payload)
            reliable_message_sender.stop()
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)
        self.assertTrue(state_machine_queue.empty())

    def test_ReliableMessage_make_sure_we_can_call_stop_twice_back_to_back(self):
        """In this test we will call stop twice in a row. Then we will make sure it still works"""
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)
        reliable_message_sender.start_loop()
        try:
            reliable_message_sender.stop()
            reliable_message_sender.stop()

            mock_message_sender.send("test", "data")
            reliable_message_sender.start(state_machine_queue.put)

            reliable_message_sender.stop()
            reliable_message_sender.stop()

            message1 = state_machine_queue.get(timeout=5.0)
            self.assertEqual(message1["type"], "test")
            self.assertEqual(message1["data"], "data")
            self.assertTrue(state_machine_queue.empty())
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)

    def test_ReliableMessage_make_sure_we_can_buffer_more_than_one_message_but_send_only_one_at_a_time(
        self,
    ):
        """In this test we will call stop twice in a row. Then we will make sure it still works"""
        state_machine_queue = Queue()
        mock_message_sender = NonCallableMock(wraps=self.MockMessageSender())
        reliable_message_sender = ReliableMessageSender(mock_message_sender)

        all_messages = [("1", "1"), ("2", "2"), ("3", "3")]

        reliable_message_sender.start_loop()
        try:
            for m in all_messages:
                mock_message_sender.send(*m)

            reliable_message_sender.start(state_machine_queue.put)
            reliable_message_sender.stop()

            m1 = state_machine_queue.get(timeout=5.0)
            e1_type, e1_data = all_messages[0]
            self.assertEqual(m1["type"], e1_type)
            self.assertEqual(m1["data"], e1_data)
            self.assertTrue(state_machine_queue.empty())
        finally:
            reliable_message_sender.stop_loop(timeout=5.0)


class TestHeartbeatMessageSender(unittest.TestCase):
    @patch("src.dr_onboard_autonomy.message_senders.time")
    def test_on_heartbeat_message(self, mock_time):
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender

        mock_time_monotonic = 3
        mock_time.monotonic.return_value = mock_time_monotonic
        mock_mqtt_client = NonCallableMock()
        mock_service_id = 1

        heartbeat_0 = Mock()
        type(heartbeat_0).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id,
            "heartbeat" : 15
        }))
        heartbeat_1 = Mock()
        type(heartbeat_1).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id,
            "heartbeat" : 17
        }))

        mock_mqtt_payload_sequence = [
            heartbeat_0,
            heartbeat_1
        ]

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )

        '''
        simulate _on_heartbeat_message being called multiple times as mqtt heartbeat messages arrive
        '''
        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            heartbeat_message_sender._on_heartbeat_message(
                client=None,
                userdata=None,
                mqtt_payload=mock_mqtt_payload
            )

        self.assertEqual(heartbeat_message_sender.heartbeat_service_id, mock_service_id)
        self.assertEqual(
            heartbeat_message_sender._latest_ground_heartbeat,
            json.loads(mock_mqtt_payload_sequence[1].payload)["heartbeat"])
        self.assertEqual(heartbeat_message_sender.last_time_confirmed, mock_time_monotonic)


    @patch("src.dr_onboard_autonomy.message_senders.time")
    def test_on_heartbeat_message_new_service_id(self, mock_time):
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender

        mock_time_monotonic_0 = 3
        mock_time_monotonic_1 = 5
        mock_time.monotonic.return_value = mock_time_monotonic_0
        mock_mqtt_client = NonCallableMock()
        mock_service_id_0 = 1
        mock_service_id_1 = 55

        heartbeat_0 = Mock()
        type(heartbeat_0).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id_0,
            "heartbeat" : 15
        }))
        heartbeat_1 = Mock()
        type(heartbeat_1).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id_0,
            "heartbeat" : 17
        }))
        heartbeat_2 = Mock()
        type(heartbeat_2).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id_1,
            "heartbeat" : 19
        }))

        mock_mqtt_payload_sequence = [
            heartbeat_0,
            heartbeat_1,
            heartbeat_2
        ]

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )

        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            if json.loads(mock_mqtt_payload.payload)["service_id"] == mock_service_id_1:
                mock_time.monotonic.return_value = mock_time_monotonic_1
            heartbeat_message_sender._on_heartbeat_message(
                client=None,
                userdata=None,
                mqtt_payload=mock_mqtt_payload
            )

        self.assertEqual(heartbeat_message_sender.heartbeat_service_id, mock_service_id_1)
        self.assertEqual(
            heartbeat_message_sender._latest_ground_heartbeat, 
            json.loads(mock_mqtt_payload_sequence[2].payload)["heartbeat"]
        )
        self.assertEqual(heartbeat_message_sender.last_time_confirmed, mock_time_monotonic_1)
        self.assertEqual(
            heartbeat_message_sender._last_heartbeat_confirmed, 
            json.loads(mock_mqtt_payload_sequence[2].payload)["heartbeat"]
        )
        self.assertEqual(
            heartbeat_message_sender._latest_ground_heartbeat, 
            json.loads(mock_mqtt_payload_sequence[2].payload)["heartbeat"]
        )


    @patch("src.dr_onboard_autonomy.message_senders.time")
    @patch("src.dr_onboard_autonomy.message_senders.RepeatTimer")
    def test_on_heartbeat_check_new_heartbeat(self, mock_repeat_timer, mock_time):
        """
        confirm Heartbeat.last_time_confirmed updates and no thresholds breached = no messages sent
        """
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender

        mock_time_monotonic_0 = 3
        mock_time.monotonic.return_value = mock_time_monotonic_0
        mock_mqtt_client = NonCallableMock()
        mock_service_id = 1

        heartbeat_0 = Mock()
        type(heartbeat_0).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id,
            "heartbeat" : 15
        }))
        heartbeat_1 = Mock()
        type(heartbeat_1).payload = PropertyMock(return_value=json.dumps({
            "service_id" : mock_service_id,
            "heartbeat" : 19
        }))


        mock_mqtt_payload_sequence = [
            heartbeat_0,
            heartbeat_1
        ]

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )
        is_called = False
        def cb(_):
            nonlocal is_called
            is_called = True
        heartbeat_message_sender.start(cb)


        for mock_mqtt_payload in mock_mqtt_payload_sequence:
            heartbeat_message_sender._on_heartbeat_message(
                client=None,
                userdata=None,
                mqtt_payload=mock_mqtt_payload
            )

        '''
        RepeatTimer is patched, so not running _on_heartbeat_check
        '''
        mock_time_monotonic_1 = 5
        mock_time.monotonic.return_value = mock_time_monotonic_1
        heartbeat_message_sender._on_heartbeat_check(message=None)

        self.assertEqual(
            heartbeat_message_sender._last_heartbeat_confirmed,
            json.loads(mock_mqtt_payload_sequence[1].payload)["heartbeat"]
        )
        self.assertEqual(heartbeat_message_sender.last_time_confirmed, mock_time_monotonic_1)
        self.assertTrue(is_called, "The to_active_state callback wasn't called")


    @patch("src.dr_onboard_autonomy.message_senders.time")
    @patch("src.dr_onboard_autonomy.message_senders.RepeatTimer")
    def test_on_heartbeat_check_thresholds_breached(self, mock_repeat_timer, mock_time):
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender

        mock_time_monotonic_0 = 3
        mock_time.monotonic.return_value = mock_time_monotonic_0
        mock_mqtt_client = NonCallableMock()

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )
        is_called = []
        def cb(message):
            nonlocal is_called
            is_called.append(message)
        heartbeat_message_sender.start(cb)

        mock_time.monotonic.return_value = 25
        heartbeat_message_sender._on_heartbeat_check(message=None)
        expected_messages = [
            {
                "type": "heartbeat_status",
                "data": {
                    "status" : "hover"
                }
            },
            {
                "type": "heartbeat_status",
                "data": {
                    "status" : "rtl"
                }
            }
        ]

        mock_time.monotonic.return_value = 40
        heartbeat_message_sender._on_heartbeat_check(message=None)

        for real, actual in zip(is_called, expected_messages):
            self.assertEqual(real, actual, "message wasn't sent to the state")


    @patch("src.dr_onboard_autonomy.message_senders.time")
    @patch("src.dr_onboard_autonomy.message_senders.RepeatTimer")
    def test_on_heartbeat_check_continue(self, mock_repeat_timer, mock_time):
        """
        continue mqtt heartbeat_status sent when previous message = hover and 
        last_time_confirmed < hover_threshold
        """
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender

        mock_time_monotonic_0 = 3
        mock_time.monotonic.return_value = mock_time_monotonic_0
        mock_mqtt_client = NonCallableMock()

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )
        actual = []
        def cb(message):
            actual.append(message)
        
        heartbeat_message_sender.start(cb)

        mock_time.monotonic.return_value = 25
        heartbeat_message_sender._on_heartbeat_check(message=None)

        expected = [
            {
                "type": "heartbeat_status",
                "data": {
                    "status" : "hover"
                }
            },
            {
                "type": "heartbeat_status",
                "data": {
                    "status" : "continue"
                }
            },
        ]


        mock_time.monotonic.return_value = 27
        '''
        _on_heartbeat_check will being running its first guard clause logic as this is the 
        first heartbeat message it will receive
        '''
        heartbeat_0 = Mock()
        type(heartbeat_0).payload = PropertyMock(return_value=json.dumps({
            "service_id" : 1,
            "heartbeat" : 15
        }))

        mock_mqtt_payload = heartbeat_0

        heartbeat_message_sender._on_heartbeat_message(
            client=None,
            userdata=None,
            mqtt_payload=mock_mqtt_payload
        )

        '''
        because a valid heartbeat was received a second ago - and we haven't breached rtl threshold,
        _on_heartbeat_check should now publish 'continue'
        '''
        mock_time.monotonic.return_value = 28
        heartbeat_message_sender._on_heartbeat_check(message=None)

        for real_msg, expect_msg in zip(actual, expected):
            self.assertEqual(real_msg,  expect_msg)


    @patch("src.dr_onboard_autonomy.message_senders.RepeatTimer")
    def test_start_stop_heartbeat(self, mock_repeat_timer):
        from src.dr_onboard_autonomy.message_senders import HeartbeatMessageSender
        
        mock_mqtt_client = NonCallableMock()

        heartbeat_message_sender = HeartbeatMessageSender(
            mqtt_client=mock_mqtt_client,
            hover_threshold=10,
            rtl_threshold=30
        )

        heartbeat_message_sender.start_heartbeat()

        mock_repeat_timer.return_value.start.assert_called_once_with(
            heartbeat_message_sender._on_heartbeat_check
        )

        heartbeat_message_sender.stop_heartbeat()

        mock_repeat_timer.return_value.stop.assert_called_once_with()


class MessageSender2TestSuite(TestReliableMessageSender):
    pass


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_message_senders2", MessageSender2TestSuite)
