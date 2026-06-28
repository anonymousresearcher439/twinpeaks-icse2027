"""
This module holds our special message queue.
"""
from collections import deque
from enum import Enum
from threading import Event, RLock
from typing import Any, Callable, Optional, Protocol, TypedDict, Iterable


class OptionalMessageKeys(TypedDict, total=False):
    done: Callable[[], None]
    message_id: int


class RequiredMessageKeys(TypedDict):
    type: str
    data: Any


class Message(RequiredMessageKeys,OptionalMessageKeys):
    pass


class MessageQueue(Protocol):
    """The namesake of this module.
    This type is used to push messages and retrieve them later in a FIFO-ish manner.
    Some messages may be overwriten by newer messages of the same type.

    The messages are added to the queue in a first-in, first-out (FIFO) manner. The order in which messages are added to the queue is the order in which they are processed and removed.

    But there's a catch: all messages are either "normal" messages or "special" messages.

    - When a normal message is added to the queue, it is simply appended to the end of the queue.
    - When a special message is added to the queue, the following steps are taken:
        1. Check if there are any other messages in the queue with the same type. If there are none, append the special message to the end of the queue.
        2. If there is a message in the queue with the same type, replace that message with the new special message.
    """
    def put(self, message: Message) -> None:
        ...
    def get(self) -> Message:
        ...
    
    def __len__(self) -> int:
        ...

class MessagePlaceholder(Protocol):
    """ This object is used to hold a place in the queue. 
    """
    def get(self) -> Message:
        ...

class SimplePlaceholder(MessagePlaceholder):
    def __init__(self, message: Message):
        self.message = message
    
    def get(self) -> Message:
        return self.message


class ReferencePlaceholder(MessagePlaceholder):
    def __init__(self, func: Callable[[], Message]):
        self.func = func
    def get(self) -> Message:
        return self.func()


class RecordState(Enum):
    """State of a message type in the lookup table."""
    MISSING = 1
    AVAILABLE = 2

class MessageLookupTable:
    class MessageRecord:
        def __init__(self):
            self.state = RecordState.MISSING
            self.data = None
        
        def update(self, data):
            self.state = RecordState.AVAILABLE
            self.data = data
        
        def get(self):
            self.state = RecordState.MISSING
            return self.data

    def __init__(self, message_types: Iterable[str]):
        self._message_types = frozenset(message_types)
        
        self._message_lookup_table = {}
        for t in self._message_types:
            self._message_lookup_table[t] = self.MessageRecord()
        
        self._lock = RLock()

    def create_ref_message(self, message: Message): # -> RMessage:
        message_type = message["type"] 
        assert message_type in self._message_types
        assert self._message_lookup_table[message_type].state == RecordState.MISSING

        self._message_lookup_table[message_type].update(message)
        return ReferencePlaceholder(lambda: self._message_lookup_table[message_type].get())
        
    def is_available(self, message_type: str) -> bool:
        assert message_type in self._message_types
        return self._message_lookup_table[message_type].state == RecordState.AVAILABLE
    
    def update(self, message: Message):
        message_type = message["type"]
        assert message_type in self._message_types
        assert self._message_lookup_table[message_type].state == RecordState.AVAILABLE
        self._message_lookup_table[message_type].update(message)


class PlaceholderFactory:
    """Given a message, create a placeholder object. This object will hold a message's place in the queue.

    Unless the type of message is special. In that case return a special kind of placeholder.
    This placeholder that will reference the most up-to-date message of it's type. This way we can update
    the refernce while holding it's place in the queue.
    """

    """These are the special message types. Newer messages of these types will
    overwrite older messages of the same type that are in the queue."""
    special_types = frozenset({
        "battery",
        "compass_hdg",
        "imu",
        "position",
        "relative_altitude",
        "velocity",
        "vision",
    })

    def __init__(self):
        self.lookup_table = MessageLookupTable(PlaceholderFactory.special_types)
    
    def make_placeholder(self, message: Message):
        message_type = message["type"]
        if message_type not in self.lookup_table._message_types:
            """This is a normal message. Return a simple placeholder.
            A simple placeholder owns the message and will return it when asked.
            """
            return SimplePlaceholder(message)
        
        if self.lookup_table.is_available(message_type):
            """this is a special type of message and there's already a message of this type in the queue.
            We need to update the existing reference. That way the ReferencePlaceholder will return
            the most up-to-date message."""
            self.lookup_table.update(message)
            return None
        # else the message is special, there isn't an older message of this type in the queue
        # so we need to create a placeholder for it
        else:
            return self.lookup_table.create_ref_message(message)



class MessageQueue(MessageQueue):
    def __init__(self):
        self.queue: deque[MessagePlaceholder] = deque()
        self.factory = PlaceholderFactory()
        self.lock = RLock()
        self.event = Event()

    def put(self, message: Message) -> None:
        with self.lock:
            placeholder = self.factory.make_placeholder(message)
            
            if placeholder is not None:
                """If the placeholder is None, then we updated an existing
                placeholder that's already in the queue.
                """
                self.queue.append(placeholder)
                self.event.set()

    def get(self) -> Message:
        while True:
            result = self._get_internal()
            if result is None:
                self.event.wait()
            else:
                return result.get()

    def _get_internal(self) -> Message:
        with self.lock:
            if not self.queue:
                self.event.clear()
                return None
            return self.queue.popleft()
    
    def __len__(self) -> int:
        with self.lock:
            return len(self.queue)