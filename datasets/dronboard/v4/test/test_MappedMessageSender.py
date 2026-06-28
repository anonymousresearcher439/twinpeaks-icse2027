#!/usr/bin/env python3

import json
import inspect
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

from sensor_msgs.msg import BatteryState


from dr_onboard_autonomy.message_senders import MappedMessageSender


class TestMappedMessageSender(unittest.TestCase):

    def test_init(self):
        name = "test"
        mapping_function = lambda x: x # identity function
        message_sender = NonCallableMock()
        mapped_sender = MappedMessageSender(name, mapping_function, message_sender)

        self.assertIsInstance(mapped_sender, MappedMessageSender)

        self.assertEqual(mapped_sender.name, name)
        self.assertEqual(mapped_sender.mapping_function, mapping_function)
        self.assertEqual(mapped_sender.message_sender, message_sender)
        self.assertIsNone(mapped_sender._to_active_state)
    
    def test_start(self):
        name = "test"
        mapping_function = lambda x: x
        original_message_sender = NonCallableMock()
        mapped_sender = MappedMessageSender(name, mapping_function, original_message_sender)

        raw_message = None
        def to_active_state(msg):
            nonlocal raw_message
            raw_message = msg
        

        mapped_sender.start(to_active_state)
        # make sure original_message_sender.start was called with mapped_sender._map_and_send
        original_message_sender.start.assert_called_once_with(mapped_sender._map_and_send)

        self.assertEqual(mapped_sender._to_active_state, to_active_state)

        example_message = {
            "data": "example",
            "type": "testing"
        }
        mapped_sender._map_and_send(example_message)
        self.assertEqual(raw_message, example_message)
    
    def test_that_value_is_mapped_and_name_is_replaced(self):
        name = "a new name"

        def mapping_function(original_data):
            return original_data + 1

        original_message_sender = NonCallableMock()

        mapped_sender = MappedMessageSender(name, mapping_function, original_message_sender)

        raw_message = None
        def to_active_state(msg):
            nonlocal raw_message
            raw_message = msg
        

        mapped_sender.start(to_active_state)
        # make sure original_message_sender.start was called with mapped_sender._map_and_send
        original_message_sender.start.assert_called_once_with(mapped_sender._map_and_send)

        self.assertEqual(mapped_sender._to_active_state, to_active_state)

        original_message = {
            "data": 0,
            "type": "the original name"
        }
        mapped_sender._map_and_send(original_message)

        expected_message = {
            "data": 1,
            "type": "a new name"
        }
        self.assertDictEqual(raw_message, expected_message)
    
    def test_that_extra_props_are_kept_in_the_message(self):
        #Also test that it works when the mapped data is falsy
        name = "a new name"

        def mapping_function(original_data):
            return False

        original_message_sender = NonCallableMock()

        mapped_sender = MappedMessageSender(name, mapping_function, original_message_sender)

        raw_message = None
        def to_active_state(msg):
            nonlocal raw_message
            raw_message = msg
        

        mapped_sender.start(to_active_state)
        # make sure original_message_sender.start was called with mapped_sender._map_and_send
        original_message_sender.start.assert_called_once_with(mapped_sender._map_and_send)

        self.assertEqual(mapped_sender._to_active_state, to_active_state)

        original_message = {
            "data": 0,
            "type": "the original name",
            "extra": "extra data"
        }
        mapped_sender._map_and_send(original_message)

        expected_message = {
            "data": False,
            "type": "a new name",
            "extra": "extra data"
        }
        self.assertDictEqual(raw_message, expected_message)
    
    def make_sure_we_filter_messages_when_mapping_func_returns_none(self):
        name = "a new name"

        def mapping_function(original_data):
            return None

        original_message_sender = NonCallableMock()

        mapped_sender = MappedMessageSender(name, mapping_function, original_message_sender)

        raw_message = None
        def to_active_state(msg):
            nonlocal raw_message
            raw_message = msg
            self.fail("This should not be called")
        

        mapped_sender.start(to_active_state)
        # make sure original_message_sender.start was called with mapped_sender._map_and_send
        original_message_sender.start.assert_called_once_with(mapped_sender._map_and_send)

        self.assertEqual(mapped_sender._to_active_state, to_active_state)

        original_message = {
            "data": 0,
            "type": "the original name",
            "extra": "extra data"
        }
        mapped_sender._map_and_send(original_message)

        self.assertIsNone(raw_message)