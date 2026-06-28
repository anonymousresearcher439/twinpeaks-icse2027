import bisect

from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass(order=True, frozen=True)
class _CallBackRecord:
    priority: int
    sequence_number: int 
    callback: Callable[[dict], Optional[str]] = field(compare=False)
    
    def __eq__(self, other):
        if not isinstance(other, _CallBackRecord):
            return False
        return (self.priority == other.priority and
                self.callback == other.callback)


class MessageHandler:
    def __init__(self):
        self.passive_handlers = {}
        self.active_handlers = {}
        self.sequence_number = 0

    def add_handler(self, message_type, callback, can_transition=True, priority=0):
        """Add a message handler.
        The message_type is a string that identifies the message type. For example, "position" or "velocity".
        The callback is a function that will be called when a message of the specified type is received.
        If is_active is True, the handler can return a value to trigger a state transition.
        If is_active is False, the handler will not be able to trigger a state transition (any return value will be ignored).

        The priority argument is used to determine the order in which handlers are called. Handlers with highest priority be called first. The highest priority (lowest value) is always at the front of the queue. Recommended that you leave this at the default value of 0. But if you need a handler to run after another handler, you can increase the priority of the second handler.
        """
        # Make a _CallBackRecord with the arguments
        callback_entry = _CallBackRecord(priority, self.sequence_number, callback)
        self.sequence_number += 1
        
        if can_transition:
            self._add_handler(self.active_handlers, message_type, callback_entry)
        else:
            self._add_handler(self.passive_handlers, message_type, callback_entry)

    def notify(self, message):
        message_type = message["type"]

        if message_type in self.passive_handlers:
            passive_callbacks = self.passive_handlers[message_type]
            for callback_entry in passive_callbacks:
                func = callback_entry.callback
                func(message)
        if message_type in self.active_handlers:
            active_callbacks = self.active_handlers[message_type]
            for callback_entry in active_callbacks:
                func = callback_entry.callback
                result = func(message)
                if result is not None:
                    return result
        return None

    def _add_handler(self, lookup_table, message_type, callback_entry):
        if message_type not in lookup_table:
            lookup_table[message_type] = []
        handlers = lookup_table[message_type]
        if callback_entry not in handlers:
            bisect.insort(handlers, callback_entry)

