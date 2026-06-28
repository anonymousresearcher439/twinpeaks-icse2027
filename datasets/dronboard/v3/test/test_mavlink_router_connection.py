from queue import Empty
import socket
import rospy
from threading import RLock, Thread
import unittest
from unittest.mock import MagicMock, Mock, patch, PropertyMock, NonCallableMagicMock

from dr_onboard_autonomy.mavlink import MavlinkRouterTCP


class TestMavlinkRouterTCP(unittest.TestCase):
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.rospy", spec=rospy)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=socket)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.Thread", spec=Thread)
    def setUp(
        self,
        mock_Thread: NonCallableMagicMock,
        mock_socket: NonCallableMagicMock,
        mock_rospy: NonCallableMagicMock
    ):
        self.mock_Thread = mock_Thread
        self.mock_socket = mock_socket

        self.mock_gained_connection_callback = Mock()
        self.mock_lost_connection_callback = Mock()
        self.mock_receive_message_callback = Mock()

        self.mavlink_router_tcp = MavlinkRouterTCP(
            gained_connection_callback=self.mock_gained_connection_callback,
            lost_connection_callback=self.mock_lost_connection_callback,
            receive_message_callback=self.mock_receive_message_callback
        )


    def test_manage_thread_shutdown(self):
        self.mavlink_router_tcp._exit_event.set()
        self.mavlink_router_tcp._receive = Mock()

        self.mavlink_router_tcp._manage_thread_shutdown(self.mavlink_router_tcp._receive)
        self.mavlink_router_tcp._receive.assert_called_once_with()

        self.mavlink_router_tcp._manage_thread_shutdown(
            self.mavlink_router_tcp._receive,
            {"arg_a": 1}
        )
        self.mavlink_router_tcp._receive.assert_called_with(arg_a=1)


    def test_close(self):
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock
        mock_socket_connected_lock = MagicMock()
        self.mavlink_router_tcp._socket_connected_lock = mock_socket_connected_lock

        self.mavlink_router_tcp.close()
        mock_socket_lock.__enter__.assert_called_once_with()
        mock_socket_connected_lock.__enter__.assert_called_once_with()
        self.assertFalse(self.mavlink_router_tcp._socket_connected)
        self.assertTrue(self.mavlink_router_tcp._exit_event.is_set())


    def test_close_socket_is_None(self):
        """This test will fail with an AttributeError if there is no guard clause against the
        _socket attribute being None before calling close"""
        mock_socket = None
        self.mavlink_router_tcp._socket = mock_socket
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock

        self.mavlink_router_tcp.close()
        mock_socket_lock.__enter__.assert_called_once_with()


    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.rospy", spec=rospy)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=socket)
    def test_create_socket_connection(
        self,
        mock_socket: NonCallableMagicMock,
        mock_rospy: NonCallableMagicMock
    ):
        # so socket connection attempt will run again
        self.mavlink_router_tcp._socket_connected = False
        self.mavlink_router_tcp._socket_lock = NonCallableMagicMock(spec=RLock())
        self.mavlink_router_tcp._socket_connected_lock = NonCallableMagicMock(spec=RLock())

        mock_gained_connection_callback_lock = MagicMock()
        mock_gained_connection_callback = Mock()
        self.mavlink_router_tcp._gained_connection_callback_lock = (
            mock_gained_connection_callback_lock)
        self.mavlink_router_tcp._gained_connection_callback = mock_gained_connection_callback

        self.mavlink_router_tcp._create_socket_connection()

        mock_socket.socket.assert_called_once_with(mock_socket.AF_INET, mock_socket.SOCK_STREAM)
        mock_socket.socket.return_value.connect.assert_called_once_with(("mavlink_router_gimbal", 5760))
        mock_socket.socket.return_value.settimeout.assert_called_once_with(None)
        self.mavlink_router_tcp._socket_lock.__enter__.assert_called_once_with()
        self.assertEqual(self.mavlink_router_tcp._socket_connected_lock.__enter__.call_count, 2)
        self.assertTrue(self.mavlink_router_tcp._socket_connected)

        mock_gained_connection_callback_lock.__enter__.assert_called_once_with()
        mock_gained_connection_callback.assert_called_once_with()


    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=socket)
    def test_create_socket_connection_already_connected(self, mock_socket: NonCallableMagicMock):
        self.mavlink_router_tcp._socket_connected = True
        self.mavlink_router_tcp._create_socket_connection()
        mock_socket.socket.assert_not_called()


    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.rospy", spec=rospy)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=socket)
    def test_create_socket_connection_error(
        self,
        mock_socket: NonCallableMagicMock,
        mock_rospy: NonCallableMagicMock
    ):
        # so socket connection attempt will run again
        self.mavlink_router_tcp._socket_connected = False

        mock_socket.socket.return_value.connect.side_effect = OSError
        self.mavlink_router_tcp._socket_lock = NonCallableMagicMock(spec=RLock())

        self.mavlink_router_tcp._create_socket_connection()

        mock_socket.socket.assert_called_once_with(
            mock_socket.AF_INET,
            mock_socket.SOCK_STREAM
        )

        self.assertEqual(self.mavlink_router_tcp._socket_lock.__enter__.call_count, 2)
        mock_socket.socket.return_value.close.assert_called_once_with()
        self.assertFalse(self.mavlink_router_tcp._socket_connected)


    def test_send_message(self):
        mock_message: bytes = b"some message"
        self.mavlink_router_tcp._socket_connected = True
        mock_socket_connected_lock = MagicMock()
        self.mavlink_router_tcp._socket_connected_lock = mock_socket_connected_lock
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket

        self.mavlink_router_tcp._send_message(mock_message)

        mock_socket_connected_lock.__enter__.assert_called_once_with()
        mock_socket.sendall.assert_called_once_with(mock_message)


    def test_send_message_socket_not_connected(self):
        mock_message: bytes = b"some message"
        try:
            type(self.mavlink_router_tcp)._socket_connected = PropertyMock(
                side_effect=(False, False, True)
            )

            mock_socket = Mock()
            self.mavlink_router_tcp._socket = mock_socket
            mock_socket_connected_lock = MagicMock()
            self.mavlink_router_tcp._socket_connected_lock = mock_socket_connected_lock
            mock_create_socket_connection = Mock()
            self.mavlink_router_tcp._create_socket_connection = mock_create_socket_connection

            self.mavlink_router_tcp._send_message(mock_message)

            self.assertEqual(mock_socket_connected_lock.__enter__.call_count, 3)
            # called once for each time _socket_connected is False
            self.assertEqual(mock_create_socket_connection.call_count, 2)
            mock_socket.sendall.assert_called_once_with(mock_message)
        finally:
            del type(self.mavlink_router_tcp)._socket_connected



    def test_send_message_exit_event_set(self):
        mock_message: bytes = b"some message"
        self.mavlink_router_tcp._socket_connected = True
        mock_socket_connected_lock = MagicMock()
        self.mavlink_router_tcp._socket_connected_lock = mock_socket_connected_lock
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        self.mavlink_router_tcp._exit_event.set()

        self.mavlink_router_tcp._send_message(mock_message)

        mock_socket_connected_lock.__enter__.assert_not_called()
        mock_socket.sendall.assert_not_called()


    def test_process_send_message_queue(self):
        mock_message = b"some outbound message"
        self.mavlink_router_tcp._outbound_message_queue.put(mock_message)
        mock_send_message = Mock()
        self.mavlink_router_tcp._send_message = mock_send_message

        self.mavlink_router_tcp._process_send_message_queue()
        mock_send_message.assert_called_once_with(mock_message)


    def test_process_send_message_handles_queue_get_Empty_exception(self):
        """checks the _send_message exits cleanly on an Empty exception from queue.get
        """
        self.mavlink_router_tcp._outbound_message_queue = Mock()
        self.mavlink_router_tcp._outbound_message_queue.get.side_effect = Empty()

        self.mavlink_router_tcp._process_send_message_queue()


    def test_send(self):
        mock_message = b"an outbound message"
        mock_outbound_message_queue = Mock()
        self.mavlink_router_tcp._outbound_message_queue = mock_outbound_message_queue

        self.mavlink_router_tcp.send(mock_message)
        mock_outbound_message_queue.put.assert_called_once_with(mock_message)


    def test_receive(self):
        mock_message = b"\xfdsome mavlink message"
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        mock_socket.recv.return_value = mock_message
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock

        mock_receive_message_callback_lock = MagicMock()
        self.mavlink_router_tcp._receive_message_callback_lock = mock_receive_message_callback_lock

        self.mavlink_router_tcp._receive()
        mock_socket_lock.__enter__.assert_called_once_with()
        mock_receive_message_callback_lock.__enter__.assert_called_once_with()
        self.mock_receive_message_callback.assert_called_once_with(mock_message)


    def test_receive_socket_disconnected_empty_message(self):
        # socket recv() returns empty bytes message if disconnected
        mock_message = b""
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        mock_socket.recv.return_value = mock_message
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock
        mock_socket_connected_lock = MagicMock()
        self.mavlink_router_tcp._socket_connected_lock = mock_socket_connected_lock
        mock_create_socket_connection = Mock()
        self.mavlink_router_tcp._create_socket_connection = mock_create_socket_connection

        mock_lost_connection_callback_lock = MagicMock()
        mock_lost_connection_callback = Mock()
        self.mavlink_router_tcp._lost_connection_callback_lock = mock_lost_connection_callback_lock
        self.mavlink_router_tcp._lost_connection_callback = mock_lost_connection_callback

        self.mavlink_router_tcp._receive()
        mock_socket_lock.__enter__.assert_called_once_with()
        self.mock_receive_message_callback.assert_not_called()

        mock_lost_connection_callback_lock.__enter__.assert_called_once_with()
        mock_lost_connection_callback.assert_called_once_with()

        self.assertEqual(mock_socket_connected_lock.__enter__.call_count, 2)
        self.assertFalse(self.mavlink_router_tcp._socket_connected)


    def test_receive_socket_not_connected(self):
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        self.mavlink_router_tcp._socket_connected = False
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock
        mock_create_socket_connection = Mock()
        self.mavlink_router_tcp._create_socket_connection = mock_create_socket_connection

        self.mavlink_router_tcp._receive()
        self.mock_receive_message_callback.assert_not_called()
        mock_create_socket_connection.assert_called_once_with()


    def test_receive_multiple_mavlink_messages_one_bytes_block(self):
        mock_message = b"\xfd\x01\x00\xfd\xfe\x43\x3a\xfd\x35\x00"
        mock_socket = Mock()
        self.mavlink_router_tcp._socket = mock_socket
        mock_socket.recv.return_value = mock_message
        mock_socket_lock = MagicMock()
        self.mavlink_router_tcp._socket_lock = mock_socket_lock

        mock_receive_message_callback_lock = MagicMock()
        self.mavlink_router_tcp._receive_message_callback_lock = mock_receive_message_callback_lock

        self.mavlink_router_tcp._receive()
        mock_socket_lock.__enter__.assert_called_once_with()
        self.assertEqual(mock_receive_message_callback_lock.__enter__.call_count, 3)
        self.assertEqual(self.mock_receive_message_callback.call_count, 3)
        self.mock_receive_message_callback.assert_any_call(b"\xfd\xfe\x43\x3a")


    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.Thread", spec=Thread)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=socket)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=RLock)
    @patch("dr_onboard_autonomy.mavlink.mavlink_router_connection.socket", spec=rospy)
    def test_init(
        self,
        mock_rospy: NonCallableMagicMock,
        mock_lock: NonCallableMagicMock,
        mock_socket: NonCallableMagicMock,
        mock_Thread: NonCallableMagicMock
    ):

        mavlink_router_tcp = MavlinkRouterTCP()

        # initial socket connection created
        mock_socket.socket.assert_called()
        mock_socket.socket.return_value.connect.assert_called()

        mock_Thread.assert_any_call(
            target=mavlink_router_tcp._manage_thread_shutdown,
            kwargs={"function_to_loop": mavlink_router_tcp._receive},
            daemon=True
        )

        mock_Thread.assert_called_with(
            target=mavlink_router_tcp._manage_thread_shutdown,
            kwargs={"function_to_loop": mavlink_router_tcp._process_send_message_queue},
            daemon=True
        )


    def test_receive_process_start(self):
        # for some reason test function name causes failure
        self.mavlink_router_tcp.start()

        self.assertEqual(self.mock_Thread.return_value.start.call_count, 2)


