from threading import Event, Thread
import unittest
from unittest.mock import patch, Mock, NonCallableMagicMock

from src.dr_onboard_autonomy.models.gimbal import Gimbal, GimbalAxes, Quaternion


class TestGimbalAbstract(unittest.TestCase):
    @patch("src.dr_onboard_autonomy.models.gimbal.Event", spec=Event)
    @patch("src.dr_onboard_autonomy.models.gimbal.Thread", spec=Thread)
    def setUp(
        self,
        mock_Thread: NonCallableMagicMock,
        mock_Event: NonCallableMagicMock
    ):
        self.mock_Thread = mock_Thread
        self.mock_Event = mock_Event

        # need to implement the abstract methods before creating an instance
        class TestGimbal(Gimbal):
            '''Provides general class overrides for tests. If a specific override is needed
            for a test, then that test will need to create its own test gimbal instance'''
            def __init__(self):
                super().__init__()
            _on_exit = Mock()
            _process_command_queue = Mock()
            _process_messages = Mock()
            receive_message_callback = Mock()
            get_gimbal_status = Mock()
            reaquire_control = Mock()
            set_attitude = Mock()
            set_neutral_attitude = Mock()

        self.test_gimbal = TestGimbal()


    def test_init(self):
        self.mock_Thread.assert_any_call(
            target=self.test_gimbal._manage_thread_shutdown,
            kwargs={"function_to_loop": self.test_gimbal._process_command_queue},
            daemon=True
        )
        self.mock_Thread.assert_called_with(
            target=self.test_gimbal._manage_thread_shutdown,
            kwargs={"function_to_loop": self.test_gimbal._process_messages},
            daemon=True
        )
        self.assertEqual(self.mock_Thread.return_value.start.call_count, 2)


    def test_manage_thread_shutdown(self):
        self.test_gimbal._manage_thread_shutdown(self.test_gimbal._process_messages)
        self.test_gimbal._process_messages.assert_called_once_with()

        self.test_gimbal._manage_thread_shutdown(self.test_gimbal._process_messages, {"arg_a": 1})
        self.test_gimbal._process_messages.assert_called_with(arg_a=1)


    def test_get_attitude(self):
        self.test_gimbal._current_attitude_lock = NonCallableMagicMock()
        test_attitude = Quaternion([0, 0, 0.7071, 0.7071])
        self.test_gimbal._current_attitude = test_attitude

        self.assertEqual(self.test_gimbal.get_attitude(), test_attitude)
        self.test_gimbal._current_attitude_lock.__enter__.assert_called_once_with()


    def test_get_gimbal_axes(self):
        self.test_gimbal._gimbal_axes = GimbalAxes.PITCH | GimbalAxes.ROLL

        self.assertIn(GimbalAxes.PITCH, self.test_gimbal.get_gimbal_axes())
        self.assertIn(GimbalAxes.ROLL, self.test_gimbal.get_gimbal_axes())


    def test_stop_communication(self):
        self.test_gimbal._on_exit = Mock()
        self.test_gimbal.stop_communication()
        self.test_gimbal._on_exit.assert_called_once_with()
        self.mock_Event.return_value.set.assert_called_once_with()


    def test_on_exit(self):
        self.test_gimbal._on_exit()