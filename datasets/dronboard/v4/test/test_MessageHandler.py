import unittest
import json
from unittest.mock import call, Mock, NonCallableMagicMock, patch

import smach

from dr_onboard_autonomy.states.collections.MessageHandler import MessageHandler




class TestMessageHandler(unittest.TestCase):
    def test_init(self):
        handler = MessageHandler()
        self.assertEqual(handler.passive_handlers, {})
        self.assertEqual(handler.active_handlers, {})
        self.assertEqual(handler.sequence_number, 0)
    
    def test_add_handler(self):
        handler = MessageHandler()
        cb = lambda x: None
        handler.add_handler("test", cb)
        # make sure the handler was added to the active_handlers
        self.assertIn("test", handler.active_handlers)
        active_handlers = handler.active_handlers["test"]
        # make sure the handler was added to the active_handlers
        cb_record = active_handlers[0]
        self.assertEqual(cb_record.priority, 0)
        self.assertEqual(cb_record.sequence_number, 0)
        self.assertEqual(cb_record.callback, cb)
        
        # make sure we don't have any passive handlers
        self.assertNotIn("test", handler.passive_handlers)
    
    def test_notify(self):
        handler = MessageHandler()
        cb = Mock()
        handler.add_handler("test", cb)
        message = {"type": "test"}
        handler.notify(message)
        cb.assert_called_once_with(message)
    
    def test_notify_with_many_callbacks(self):
        handler = MessageHandler()
        all_callbacks = [Mock() for _ in range(10)]
        for cb in all_callbacks:
            cb.return_value = None
            handler.add_handler("test", cb)
        
        message = {"type": "test"}
        handler.notify(message)
        for cb in all_callbacks:
            cb.assert_called_once_with(message)
    
    def test_notify_with_many_callbacks_and_many_message_types(self):
        handler = MessageHandler()
        called_callbacks = [Mock() for _ in range(10)]
        for cb in called_callbacks:
            cb.return_value = None
            handler.add_handler("test", cb)
        
        not_called_callbacks = [Mock() for _ in range(10)]
        for cb in not_called_callbacks:
            cb.return_value = None
            handler.add_handler("not_test", cb)
        
        message = {"type": "test"}
        handler.notify(message)
        for cb in called_callbacks:
            cb.assert_called_once_with(message)
        for cb in not_called_callbacks:
            cb.assert_not_called()
    
    def test_notify_with_active_handler_returning_a_value(self):
        handler = MessageHandler()
        cb = Mock()
        cb.return_value = "my outcome"
        handler.add_handler("test", cb)
        message = {"type": "test"}
        result = handler.notify(message)
        self.assertEqual(result, "my outcome")
    
    def test_notify_where_passive_handlers_are_called(self):
        handler = MessageHandler()
        cb = Mock()
        handler.add_handler("test", cb, can_transition=False)
        message = {"type": "test"}
        handler.notify(message)
        cb.assert_called_once_with(message)
    
    def test_notify_where_passive_are_called_and_active_are_called(self):
        handler = MessageHandler()
        active_cb = Mock()
        handler.add_handler("test", active_cb)
        passive_cb = Mock()
        handler.add_handler("test", passive_cb, can_transition=False)
        message = {"type": "test"}
        handler.notify(message)
        active_cb.assert_called_once_with(message)
        passive_cb.assert_called_once_with(message)
    
    def test_notify_where_active_handler_returns_a_value(self):
        """Make sure we ignore the return value from the passive handler when the active handler returns a value.
        """
        handler = MessageHandler()
        active_cb = Mock()
        active_cb.return_value = "expected_outcome"
        handler.add_handler("test", active_cb)
        passive_cb = Mock()
        passive_cb.return_value = "ignored outcome"
        handler.add_handler("test", passive_cb, can_transition=False)
        message = {"type": "test"}
        result = handler.notify(message)
        self.assertEqual(result, "expected_outcome")
    
    def test_notify_order_of_callbacks(self):
        """Make sure the callbacks are called in the order they were added."""
        handler = MessageHandler()
        active_call_order = []
        def make_callback(i):
            def cb(_message):
                active_call_order.append(i)
            return cb
        for i in range(10):
            handler.add_handler("test", make_callback(i))
        
        passive_call_order = []
        def make_passive_callback(i):
            def cb(_message):
                passive_call_order.append(i)
            return cb
        for i in range(10):
            handler.add_handler("test", make_passive_callback(i), can_transition=False)
        
        message = {"type": "test"}
        handler.notify(message)
        self.assertEqual(passive_call_order, list(range(10)))
        self.assertEqual(active_call_order, list(range(10)))

    def test_overall_order_of_callbacks_passive_then_active(self):
        """Make sure the callbacks are called in the order they were added."""
        handler = MessageHandler()
        all_call_order = []
        active_call_order = []
        def make_callback(i):
            def cb(_message):
                all_call_order.append(i)
            return cb
        
        for i in range(10):
            is_active = i < 5 # first 5 are active but last 5 are passive
            handler.add_handler("test", make_callback(i), can_transition=is_active)
        expected_order = [
            5,
            6,
            7,
            8,
            9,
            0,
            1,
            2,
            3,
            4,
        ]
        message = {"type": "test"}
        handler.notify(message)
        self.assertEqual(all_call_order, expected_order)

    def test_active_callback_order_with_priorities(self):
        """Make sure the callbacks are called in the order such that

        1. The highest priority are called first
        2. equal priority are called in the order they were added
        """
        # we will add 10 callbacks the first 5 we add will have priority 10
        # the last 5 will have priority 0 (the default)

        handler = MessageHandler()
        all_call_order = []
        def make_callback(i):
            def cb(_message):
                all_call_order.append(i)
            return cb

        for i in range(10):
            if i < 5:
                handler.add_handler("test", make_callback(i), priority=10)
            else:
                handler.add_handler("test", make_callback(i))
        expected_order = [5, 6, 7, 8, 9, 0, 1, 2, 3, 4]
        message = {"type": "test"}
        handler.notify(message)
        self.assertEqual(all_call_order, expected_order)
    
    def test_passive_callbacks_with_priority(self):
        handler = MessageHandler()
        all_call_order = []
        def make_callback(i):
            def cb(_message):
                all_call_order.append(i)
            return cb

        for i in range(10):
            if i < 5:
                handler.add_handler("test", make_callback(i), can_transition=False, priority=10)
            else:
                handler.add_handler("test", make_callback(i), can_transition=False)
        expected_order = [5, 6, 7, 8, 9, 0, 1, 2, 3, 4]
        message = {"type": "test"}
        handler.notify(message)
        self.assertEqual(all_call_order, expected_order)
    
    def test_passive_callbacks_with_priority_and_active_callbacks(self):
        handler = MessageHandler()
        all_call_order = []
        def make_callback(i):
            def cb(_message):
                all_call_order.append(i)
            return cb

        # so for this test:
        # we will have 10 active callbacks
        # and 10 passive callbacks
        # the first 5 passive callbacks will have priority 10
        # the last 5 passive callbacks will have priority 0
        # the first 5 active callbacks will have priority 10
        # the last 5 active callbacks will have priority 0

        for i in range(20):
            if i < 10:
                # these are the active callbacks
                priority = 10
                if i >= 5:
                    priority = 0
                handler.add_handler("test", make_callback(i), can_transition=True, priority=priority)
            else:
                # these are the passive callbacks
                priority = 10
                if i >= 15:
                    priority = 0
                handler.add_handler("test", make_callback(i), can_transition=False, priority=priority)
        expected_order = [
            # pasive callbacks first
            # passive priority 0
            15,
            16,
            17,
            18,
            19,
            # passive priority 10
            10,
            11,
            12,
            13,
            14,
            # active callbacks
            # active priority 0
            5,
            6,
            7,
            8,
            9,
            # active priority 10
            0,
            1,
            2,
            3,
            4,
        ]
        # now let's add lots of callbacks for unhandled message types
        other_message_types = ["not_test", "other", "another"]
        all_uncalled_callbacks = []
        for msg_type in other_message_types:
            for i in range(10):
                is_active = i < 5
                if i % 2 == 0:
                    priority = 0
                else:
                    priority = 10
                cb = Mock()
                cb.return_value = None
                handler.add_handler(msg_type, cb, can_transition=is_active, priority=priority)
                all_uncalled_callbacks.append(cb)
        
        message = {"type": "test"}
        handler.notify(message)
        self.assertEqual(all_call_order, expected_order)
        for cb in all_uncalled_callbacks:
            cb.assert_not_called()