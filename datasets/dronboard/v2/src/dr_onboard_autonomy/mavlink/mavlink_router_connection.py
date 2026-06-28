from queue import Empty, Queue
import rospy
import socket
from threading import Event, RLock, Thread
from typing import Callable, Optional, Mapping

from dr_onboard_autonomy.models.connection import ExternalConnection


class MavlinkRouterTCP(ExternalConnection):
    """The required callback functions default to empty logic. A callable for each callback can be
    provided during initialization or with each respective set_*_callback function
    """
    _MAVLINK_FRAME_DELIMITER: bytes = b"\xfd"

    def __init__(
        self,
        gained_connection_callback: Callable[[], None]=lambda: None,
        lost_connection_callback: Callable[[], None]=lambda: None,
        receive_message_callback: Callable[[bytes], None]=lambda message: None,
        # 512 is the next power of 2 larger than 280 - the max size of a mavlink2 payload
        receive_buffer_size: int=512
    ):
        """Creates a new connection to the Mavlink Router TCP server and provides an interface to 
        send and receive bytes messages
        """
        super().__init__()

        # don't need to use callback locks because only one thread exists at this point
        self._gained_connection_callback = gained_connection_callback
        self._lost_connection_callback = lost_connection_callback
        self._receive_message_callback = receive_message_callback
        self._receive_buffer_size = receive_buffer_size

        self._exit_event: Event = Event()
        self._outbound_message_queue: Queue = Queue()
        self._socket_connected: bool = False
        self._socket_connected_lock: RLock = RLock()
        self._socket: Optional[socket.socket] = None
        self._socket_lock: RLock = RLock()

        self._create_socket_connection()

        self._receive_messages_thread: Thread = Thread(
            target=self._manage_thread_shutdown,
            kwargs={"function_to_loop": self._receive},
            daemon=True
        )
        self._process_send_message_queue_thread: Thread = Thread(
            target=self._manage_thread_shutdown,
            kwargs={"function_to_loop": self._process_send_message_queue},
            daemon=True
        )


    def close(self):
        """Shuts down all message processing threads and closes the socket connection
        """
        self._exit_event.set()

        with self._socket_lock:
            if self._socket is not None:
                self._socket.close()

        with self._socket_connected_lock:
            self._socket_connected = False


    def send(self, message: bytes):
        """Places message in a FIFO queue to be processed by a sender thread
        """
        self._outbound_message_queue.put(message)


    def start(self):
        self._receive_messages_thread.start()
        self._process_send_message_queue_thread.start()


    def _create_socket_connection(self):
        """Creates TCP a socket connection to the mavlink router service. Is meant to be run in its
        own thread as the process will block while trying to create the connection"""
        try:
            with self._socket_lock:
                # once the _socket_lock is released, we need to check whether the socket has already
                # regained connection in another thread
                with self._socket_connected_lock:
                    if self._socket_connected:
                        return
                self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 5760 is the mavlink router TCP server port
                self._socket.connect(('mavlink_router', 5760))
                with self._socket_connected_lock:
                    self._socket_connected = True
                with self._gained_connection_callback_lock:
                    self._gained_connection_callback()
            rospy.loginfo("MavlinkRouterTCP - created socket connection to gimbal")
        except Exception as e:
            rospy.logerr(f"MavlinkRouterTCP - failed to connect to TCP socket with error: {e}")
            with self._socket_lock:
                self._socket.close()


    def _manage_thread_shutdown(self, function_to_loop: Callable, kwargs: Optional[Mapping]=None):
        '''Loops indefinitely on function_to_loop until exit event triggers break
        '''
        while True:
            if kwargs is not None:
                function_to_loop(**kwargs)
            else:
                function_to_loop()

            if self._exit_event.is_set():
                break


    def _process_send_message_queue(self):
        """Processes messages in the outbound message queue
        """
        try:
            self._send_message(self._outbound_message_queue.get(timeout=10))
        except Empty:
            rospy.logdebug("MavlinkRouterTCP - outbound message .get timed out after 10s")


    def _send_message(self, message: bytes):
        """Guarantees a message will be sent by continually attempting to reopen a lost socket 
        connection until successful. Then sends the provided message
        """
        if self._exit_event.is_set():
            return
        with self._socket_connected_lock:
            if not self._socket_connected:
                self._create_socket_connection()
                self._send_message(message)
                return

        self._socket.sendall(message)


    def _receive(self):
        """Receives a socket bytes block of 'receive_buffer_size'. The bytes block is split into
        separate mavlink messages if necessary. Each message is passed to _receive_message_callback
        """
        with self._socket_lock:
            bytes_block: bytes = self._socket.recv(self._receive_buffer_size)

        if not bytes_block:
            with self._lost_connection_callback_lock:
                self._lost_connection_callback()
            self._create_socket_connection()
            return

        message_list = bytes_block.split(self._MAVLINK_FRAME_DELIMITER)

        for message in message_list:
            # don't pass an empty bytes message
            if message:
                with self._receive_message_callback_lock:
                    self._receive_message_callback(self._MAVLINK_FRAME_DELIMITER + message)
