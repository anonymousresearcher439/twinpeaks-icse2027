from threading import Thread
import unittest
from unittest.mock import MagicMock, Mock, NonCallableMagicMock, patch

from pymavlink.dialects.v20 import common as mavlink2

from dr_onboard_autonomy.mavlink import (
    HeartbeatSender2,
    MavlinkNode,
    MavlinkRouterTCP,
    MavState,
    MavType,
    SystemId,
    ComponentId
)


class TestHeartbeatSender2(unittest.TestCase):
    @patch("dr_onboard_autonomy.mavlink.heartbeat.Thread", spec=Thread)
    def setUp(self, mock_Thread: NonCallableMagicMock):
        self.mock_communication_channel = Mock(spec=MavlinkRouterTCP)
        self.mock_Thread = mock_Thread
        self.mock_send_frequency = 2.0

        self.heartbeat_sender = HeartbeatSender2(
            communication_channel=self.mock_communication_channel,
            controller=MavlinkNode(
                system_id=SystemId(1),
                component_id=ComponentId(MavType.ONBOARD_CONTROLLER.value)
            ),
            send_frequency=self.mock_send_frequency
        )


    def test_init(self):
        self.assertEqual(
            self.heartbeat_sender._mavlink_protocol_handler.srcSystem,
            1
        )
        self.assertEqual(
            self.heartbeat_sender._mavlink_protocol_handler.srcComponent,
            MavType.ONBOARD_CONTROLLER.value
        )

        self.heartbeat_sender.mav_state = MavState.UNINIT.value

        self.mock_Thread.assert_called_once_with(
            target=self.heartbeat_sender._manage_thread_shutdown,
            kwargs={
                "function_to_loop": self.heartbeat_sender._send_heartbeat_at_frequency,
                "kwargs":{"frequency":self.mock_send_frequency}
            },
            daemon=True
        )


    def test_manage_thread_shutdown(self):
        self.heartbeat_sender._exit_event.set()
        mock_thread_function = Mock()

        self.heartbeat_sender._manage_thread_shutdown(mock_thread_function)
        mock_thread_function.assert_called_once_with()

        self.heartbeat_sender._manage_thread_shutdown(
            mock_thread_function,
            {"arg_a": 1}
        )
        mock_thread_function.assert_called_with(arg_a=1)


    def test_stop(self):
        self.heartbeat_sender.stop()

        self.assertTrue(self.heartbeat_sender._exit_event.is_set())
        self.mock_communication_channel.close.assert_called_once_with()


    @patch("dr_onboard_autonomy.mavlink.heartbeat.sleep")
    def test_send_heartbeat_at_frequency(self, mock_sleep: MagicMock):
        mock_send_heartbeat = Mock()
        self.heartbeat_sender._send_heartbeat = mock_send_heartbeat
        self.heartbeat_sender._send_heartbeat_at_frequency(frequency=4)

        mock_send_heartbeat.assert_called_once_with()
        mock_sleep.assert_called_once_with(1/4)


    def test_send_heartbeat(self):
        mock_pack_heartbeat = Mock()
        mock_pack_heartbeat.return_value = b"a heartbeat message"
        self.heartbeat_sender._pack_heartbeat = mock_pack_heartbeat

        self.heartbeat_sender._send_heartbeat()

        self.mock_communication_channel.send.assert_called_once_with(b"a heartbeat message")
        mock_pack_heartbeat.assert_called_once_with()


    def test_pack_heartbeat(self):
        mock_mav_state = MavState.BOOT.value
        self.heartbeat_sender.mav_state = mock_mav_state

        heartbeat_bytes_message = self.heartbeat_sender._pack_heartbeat()

        self.assertEqual(
            heartbeat_bytes_message,
            mavlink2.MAVLink_heartbeat_message(
                type=MavType.ONBOARD_CONTROLLER.value,
                autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mock_mav_state,
                mavlink_version=0
            ).pack(self.heartbeat_sender._mavlink_protocol_handler)
        )


    def test_mav_state_property(self):
        mock_mav_state_lock = MagicMock()
        self.heartbeat_sender._mav_state_lock = mock_mav_state_lock

        self.heartbeat_sender.mav_state = MavState.CALIBRATING

        self.assertEqual(self.heartbeat_sender.mav_state, MavState.CALIBRATING)
        self.assertEqual(mock_mav_state_lock.__enter__.call_count, 2)


    def test_start_heartbeat(self):
        mock_send_heartbeat_at_frequency_thread = Mock()
        self.heartbeat_sender._send_heartbeat_at_frequency_thread = (
            mock_send_heartbeat_at_frequency_thread
        )
        mock_communication = Mock()
        self.heartbeat_sender._communication = mock_communication

        self.heartbeat_sender.start()

        mock_communication.start.assert_called_once_with()
        mock_send_heartbeat_at_frequency_thread.start.assert_called_once_with()