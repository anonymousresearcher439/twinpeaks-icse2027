import unittest
from unittest.mock import MagicMock, Mock


from src.dr_onboard_autonomy.models import ExternalConnection


class TestExternalConnection(unittest.TestCase):
    def setUp(self):
        class SomeConnection(ExternalConnection):
            def __init__(self):
                super().__init__()
            close = Mock()
            send = Mock()
            start = Mock()

        self.some_connection = SomeConnection()


    def test_init(self):
        self.assertIsNone(self.some_connection._gained_connection_callback())
        self.assertIsNone(self.some_connection._lost_connection_callback())
        self.assertIsNone(self.some_connection._receive_message_callback(b'a message'))


    def test_set_gained_connection_callback(self):
        mock_callback = Mock()
        mock_callback_lock = MagicMock()
        self.some_connection._gained_connection_callback_lock = mock_callback_lock

        self.some_connection.set_gained_connection_callback(mock_callback)

        self.assertEqual(self.some_connection._gained_connection_callback, mock_callback)
        mock_callback_lock.__enter__.assert_called_once_with()


    def test_set_lost_connection_callback(self):
        mock_callback = Mock()
        mock_callback_lock = MagicMock()
        self.some_connection._lost_connection_callback_lock = mock_callback_lock

        self.some_connection.set_lost_connection_callback(mock_callback)

        self.assertEqual(self.some_connection._lost_connection_callback, mock_callback)
        mock_callback_lock.__enter__.assert_called_once_with()


    def test_set_receive_message_callback(self):
        mock_callback = Mock()
        mock_callback_lock = MagicMock()
        self.some_connection._receive_message_callback_lock = mock_callback_lock

        self.some_connection.set_receive_message_callback(mock_callback)

        self.assertEqual(self.some_connection._receive_message_callback, mock_callback)
        mock_callback_lock.__enter__.assert_called_once_with()