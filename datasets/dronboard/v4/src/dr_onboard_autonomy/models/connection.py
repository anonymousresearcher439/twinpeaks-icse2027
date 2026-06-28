from abc import ABC, abstractmethod
from threading import Lock
from typing import Any, Callable


class ExternalConnection(ABC):
    """Abstract class defining the interface for various external connections. It is expected that
    any subclass will implement a receiver that calls _receive_message_callback with the received
    message. It is also expected that the _gained_connection_callback and _lost_connection_callback
    functions will be called when a connection is acquired or lost repectively. 
    """
    def __init__(self):
        self._gained_connection_callback_lock = Lock()
        self._gained_connection_callback: Callable[[], None] = lambda: None

        self._lost_connection_callback_lock = Lock()
        self._lost_connection_callback: Callable[[], None] = lambda: None

        self._receive_message_callback_lock = Lock()
        self._receive_message_callback: Callable[[Any], None] = lambda message: None


    @abstractmethod
    def close(self):
        """Shuts down all threads and open connections"""


    @abstractmethod
    def send(self, message: Any):
        """Sends the provided message"""


    def set_gained_connection_callback(self, callback: Callable[[], None]):
        """Sets or updates the function called when the connection is acquired"""
        with self._gained_connection_callback_lock:
            self._gained_connection_callback = callback


    def set_lost_connection_callback(self, callback: Callable[[], None]):
        """Sets or updates the function called when the connection is lost"""
        with self._lost_connection_callback_lock:
            self._lost_connection_callback = callback


    def set_receive_message_callback(self, callback: Callable[[Any], None]):
        """Sets or updates the function called with a bytes message upon receipt of that message"""
        with self._receive_message_callback_lock:
            self._receive_message_callback = callback


    @abstractmethod
    def start(self):
        """Kicks off the receiveing and processing of messages"""


