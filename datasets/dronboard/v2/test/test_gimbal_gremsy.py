import array
import math
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink.dialects.v20 import ardupilotmega as mavlink2_mega
from queue import Empty, Queue
from threading import Event, Lock, RLock, Thread
from typing import Mapping
import unittest
from unittest.mock import MagicMock, Mock, NonCallableMagicMock, patch

from dr_onboard_autonomy.models.gimbal import GimbalAxes, GimbalStatus, Quaternion
from dr_onboard_autonomy.gimbal.gimbal_gremsy import (
    GimbalGremsy,
    _CmdProcessMode,
    _CmdType,
    _MountConfigure,
    _ResetGimbal,
    _SetAttitude,
)
from dr_onboard_autonomy.mavlink import ComponentId, MavlinkNode, MavlinkRouterTCP, SystemId


class TestGimbalGremsy(unittest.TestCase):
    @patch("dr_onboard_autonomy.gimbal.gimbal_gremsy.MavlinkRouterTCP", spec=MavlinkRouterTCP)
    @patch("dr_onboard_autonomy.gimbal.gimbal_gremsy.RLock", spec=RLock)
    @patch("dr_onboard_autonomy.models.gimbal.Lock", spec=Lock)
    @patch("dr_onboard_autonomy.models.gimbal.Queue", spec=Queue)
    @patch("dr_onboard_autonomy.models.gimbal.Event", spec=Event)
    @patch("dr_onboard_autonomy.models.gimbal.Thread", spec=Thread)
    def setUp(
        self,
        mock_Thread_abstract: NonCallableMagicMock,
        mock_Event: NonCallableMagicMock,
        mock_Queue: NonCallableMagicMock,
        mock_Lock_abstract: NonCallableMagicMock,
        mock_Lock_mavlink: NonCallableMagicMock,
        mock_mavlink_router_tcp: NonCallableMagicMock
    ):
        self.mock_Queue_abstract = mock_Queue
        self.mock_Event_abstract = mock_Event
        self.mock_connection = mock_mavlink_router_tcp

        self.mock_controller_node = MavlinkNode(SystemId(1), ComponentId(1))
        self.gimbal_gremsy = GimbalGremsy(
            controller=self.mock_controller_node,
            gimbal_axes=GimbalAxes.PITCH
        )


    def test_init(self):
        self.assertEqual(self.gimbal_gremsy._mavlink_protocol_handler.srcSystem, SystemId(1))
        self.assertEqual(self.gimbal_gremsy._mavlink_protocol_handler.srcComponent, ComponentId(1))
        
        self.mock_Queue_abstract.return_value.put.assert_called_with(_MountConfigure())

        self.mock_connection.assert_called_once_with(
            gained_connection_callback=self.gimbal_gremsy._gained_connection,
            lost_connection_callback=self.gimbal_gremsy._lost_connection,
            receive_message_callback=self.gimbal_gremsy.receive_message_callback
        )

        self.mock_connection.return_value.start.assert_called_once_with()


    def test_process_command_queue(self):
        self.gimbal_gremsy._gimbal_status = GimbalStatus.READY
        mock_command_queue = Mock()
        mock_pack_set_attitude = Mock()
        mock_encoded_message: bytes = "some message".encode("UTF-8")
        mock_pack_set_attitude.return_value = mock_encoded_message

        mock_message: _SetAttitude = _SetAttitude(
            attitude=(0, 0, 0, 1)
        )
        mock_command_queue.get.return_value = mock_message

        self.gimbal_gremsy._command_queue = mock_command_queue
        self.gimbal_gremsy._command_to_packer_map = {
            _SetAttitude.command_type: mock_pack_set_attitude
        }
        self.gimbal_gremsy._process_command_queue()

        mock_command_queue.get.assert_called_once_with(timeout=1)
        self.assertEqual(
            self.mock_connection.return_value.send.call_args[0][0],
            mock_encoded_message
        )


    def test_process_command_queue_Empty_exception(self):
        mock_command_queue = Mock()
        mock_command_queue.get.side_effect = Empty

        self.gimbal_gremsy._command_queue = mock_command_queue
        self.gimbal_gremsy._process_command_queue()

        mock_command_queue.get.assert_called_once_with(timeout=1)
        self.mock_connection.return_value.send.assert_not_called()


    def test_process_command_queue_gimbal_status_not_ready(self):
        self.gimbal_gremsy._gimbal_status = GimbalStatus.NOT_CONNECTED
        mock_command_queue = Mock()

        # _SetAttitude command process mode is WAIT_FOR_READY
        mock_message: _SetAttitude = _SetAttitude(
            attitude=(0, 0, 0, 1)
        )
        mock_command_queue.get.return_value = mock_message

        self.gimbal_gremsy._command_queue = mock_command_queue

        self.gimbal_gremsy._process_command_queue()

        self.gimbal_gremsy._gimbal_status_lock.__enter__.assert_called_once_with()
        self.mock_connection.return_value.send.assert_not_called()


    def test_process_command_queue_gimbal_status_not_ready_mode_always(self):
        """Tests flow when processing a command with command process mode == ALWAYS
        """
        self.gimbal_gremsy._gimbal_status = GimbalStatus.CONNECTED_BUT_NOT_IN_CONTROL
        mock_command_queue = Mock()
        mock_pack_message = Mock()
        mock_encoded_message: bytes = "some message".encode("UTF-8")
        mock_pack_message.return_value = mock_encoded_message

        mock_message: _ResetGimbal = _ResetGimbal()
        mock_message.command_process_mode = _CmdProcessMode.ALWAYS
        mock_command_queue.get.return_value = mock_message
        self.gimbal_gremsy._command_to_packer_map = {
            _ResetGimbal.command_type: mock_pack_message
        }

        self.gimbal_gremsy._command_queue = mock_command_queue
        self.gimbal_gremsy._process_command_queue()

        self.mock_connection.return_value.send.assert_called()


    def test_pack_do_mount_configure(self):
        msg_do_mount_configure=_MountConfigure()

        do_mount_configure_bytes_message = self.gimbal_gremsy._pack_do_mount_configure(
            msg=msg_do_mount_configure
        )

        self.assertEqual(
            do_mount_configure_bytes_message,
            mavlink2.MAVLink_command_long_message(
                target_system=self.gimbal_gremsy._gimbal_system_id,
                target_component=self.gimbal_gremsy._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONFIGURE,
                confirmation=0,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                param5=_MountConfigure().roll_mode.value,
                param6=_MountConfigure().pitch_mode.value,
                param7=_MountConfigure().yaw_mode.value
            ).pack(self.gimbal_gremsy._mavlink_protocol_handler)
        )


    def test_receive_message_callback(self):
        mock_message = "some mock message".encode("UTF-8")
        mock_decoded_message = Mock()
        mock_decoded_message.get_msgId.return_value = 1000
        mock_mavlink_protocol_handler = Mock()
        mock_mavlink_protocol_handler.decode.return_value = mock_decoded_message

        self.gimbal_gremsy._mavlink_protocol_handler = mock_mavlink_protocol_handler
        self.gimbal_gremsy.receive_message_callback(mock_message)

        mock_mavlink_protocol_handler.decode.assert_called_once_with(array.array('B', mock_message))
        self.gimbal_gremsy._received_messages_queue.put.assert_called_with(
            (
                1000,
                mock_decoded_message
            )
        )


    def test_receive_message_callback_mavlink2_common_decode_MAVError(self):
        mock_message = "some mock message".encode("UTF-8")
        mock_decoded_message = Mock()
        mock_decoded_message.get_msgId.return_value = 1000
        mock_mavlink_protocol_handler = Mock()
        mock_mavlink_protocol_handler.decode.side_effect = mavlink2.MAVError(msg="some error")
        mock_mavlink_protocol_handler_mega = Mock()
        mock_mavlink_protocol_handler_mega.decode.return_value = mock_decoded_message

        self.gimbal_gremsy._mavlink_protocol_handler = mock_mavlink_protocol_handler
        self.gimbal_gremsy._mavlink_protocol_handler_mega = mock_mavlink_protocol_handler_mega

        self.gimbal_gremsy.receive_message_callback(mock_message)

        mock_mavlink_protocol_handler_mega.decode.assert_called_once_with(
            array.array('B', mock_message)
        )
        self.gimbal_gremsy._received_messages_queue.put.assert_called_with(
            (
                1000,
                mock_decoded_message
            )
        )


    def test_receiver_message_callback_mavlink2_mega_decode_MAVError(self):
        self.gimbal_gremsy._received_messages_queue = Mock()
        mock_message = "some mock message".encode("UTF-8")
        mock_mavlink_protocol_handler = Mock()
        mock_mavlink_protocol_handler.decode.side_effect = mavlink2.MAVError(msg="some error")
        mock_mavlink_protocol_handler_mega = Mock()
        mock_mavlink_protocol_handler_mega.decode.side_effect = mavlink2_mega.MAVError(
            msg="some other error"
        )

        self.gimbal_gremsy._mavlink_protocol_handler = mock_mavlink_protocol_handler
        self.gimbal_gremsy._mavlink_protocol_handler_mega = mock_mavlink_protocol_handler_mega

        self.gimbal_gremsy.receive_message_callback(mock_message)

        self.gimbal_gremsy._received_messages_queue.put.assert_not_called()


    def test_process_messages(self):
        mock_gimbal_device_attitue_status_message = Mock()
        self.gimbal_gremsy._received_messages_queue.get.return_value = (
            (1234, mock_gimbal_device_attitue_status_message)
        )
        mock_receive_message_callback = Mock()
        self.gimbal_gremsy._received_message_to_callback = {
            1234 : mock_receive_message_callback
        }

        self.gimbal_gremsy._process_messages()

        mock_receive_message_callback.assert_called_once_with(
            mock_gimbal_device_attitue_status_message
        )


    def test_receive_mount_status(self):
        mock_message = mavlink2_mega.MAVLink_mount_status_message(
            target_component=ComponentId(1),
            target_system=SystemId(1),
            pointing_a=9000, # pitch
            pointing_b=4500, # roll
            pointing_c=0 # yaw
        )

        self.gimbal_gremsy._receive_mount_status(mock_message)

        self.gimbal_gremsy._current_attitude_lock.__enter__.assert_not_called()
        self.assertIsNone(self.gimbal_gremsy._current_attitude)

        # NOTE: using the mount_orientation message to set current attitude instead of mount_status
        # self.gimbal_gremsy._current_attitude_lock.__enter__.assert_called_once_with()
        # self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[0], 0.270598, 6)
        # self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[1], 0.653281, 6)
        # self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[2], 0.270598, 6)
        # self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[3], 0.653281, 6)


    def test_pack_do_mount_control(self):
        # 135 degree rotation about the y-axis (pitch)
        msg_set_attitude=_SetAttitude(
            attitude=(0, 0.9238795, 0, 0.3826834)
        )

        expected_axis_angle_deg = self.gimbal_gremsy._quaternion_to_axis_angle(
            Quaternion((0, 0.9238795, 0, 0.3826834))
        )

        do_mount_control_bytes_message = self.gimbal_gremsy._pack_do_mount_control(
            msg=msg_set_attitude
        )

        self.assertEqual(
            do_mount_control_bytes_message,
            mavlink2.MAVLink_command_long_message(
                target_system=self.gimbal_gremsy._gimbal_system_id,
                target_component=self.gimbal_gremsy._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONTROL,
                confirmation=0,
                param1=expected_axis_angle_deg[1],# pitch
                param2=expected_axis_angle_deg[0], # roll
                param3=expected_axis_angle_deg[2], # yaw
                param4=0,
                param5=0,
                param6=0,
                param7=0
            ).pack(self.gimbal_gremsy._mavlink_protocol_handler)
        )


    def test_receive_heartbeat(self):
        mock_message = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_GIMBAL,
            autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink2.MAV_STATE_ACTIVE,
            mavlink_version=0
        )
        mock_message._header.srcComponent = mavlink2.MAV_COMP_ID_GIMBAL

        self.gimbal_gremsy._receive_heartbeat(mock_message)

        self.gimbal_gremsy._gimbal_status_lock.__enter__.assert_called_once_with()
        self.assertEqual(self.gimbal_gremsy._gimbal_status, GimbalStatus.READY)


    def test_set_default_mount_configuration(self):
        command_queue_mock = Mock()
        self.gimbal_gremsy._command_queue = command_queue_mock

        self.gimbal_gremsy._set_default_mount_configuration()

        self.assertEqual(_MountConfigure.command_type, _CmdType.MOUNT_CONFIGURE)

        command_queue_mock.put.assert_called_once_with(_MountConfigure())


    def test_on_exit(self):
        self.gimbal_gremsy._on_exit()

        self.mock_connection.return_value.close.assert_called_once_with()


    def test_pack_set_neutral_attitude(self):
        msg_reset_gimbal=_ResetGimbal()
        self.gimbal_gremsy._current_attitude = Quaternion((0, 0.3826834, 0, 0.9238795))
        mock_current_attitude_lock = MagicMock()
        self.gimbal_gremsy._current_attitude_lock = mock_current_attitude_lock

        set_neutral_attitude_bytes_message = self.gimbal_gremsy._pack_set_neutral_attitude(
            msg=msg_reset_gimbal
        )

        self.assertEqual(
            set_neutral_attitude_bytes_message,
            mavlink2.MAVLink_command_long_message(
                target_system=self.gimbal_gremsy._gimbal_system_id,
                target_component=self.gimbal_gremsy._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONTROL,
                confirmation=0,
                param1=-44.999996, # pitch
                param2=0, # roll
                param3=0, # yaw
                param4=0,
                param5=0,
                param6=0,
                param7=0
            ).pack(self.gimbal_gremsy._mavlink_protocol_handler)
        )
        mock_current_attitude_lock.__enter__.assert_called_once_with()


    def test_get_gimbal_status(self):
        mock_gimbal_status_lock = MagicMock()
        self.gimbal_gremsy._gimbal_status_lock = mock_gimbal_status_lock

        self.gimbal_gremsy._gimbal_status = GimbalStatus.NOT_CONNECTED

        self.assertEqual(self.gimbal_gremsy.get_gimbal_status(), GimbalStatus.NOT_CONNECTED)

        mock_gimbal_status_lock.__enter__.assert_called_once_with()


    def test_reaquire_control(self):
        self.assertIsNone(self.gimbal_gremsy.reaquire_control())


    def test_set_attitude(self):
        self.gimbal_gremsy._do_mount_configure_status_complete = True

        test_cases = [
            {
                "current_attitude": Quaternion((0, -0.7071, 0, 0.7071)),
                "set_attitude": Quaternion((0, 0.7071, 0, 0.7071)),
                "expected_difference": Quaternion((0, 1, 0, 0))
            },
            {
                "current_attitude": Quaternion((
                    0,
                    -0.3866309637136508,
                    0,
                    0.9222342741199001
                )),
                "set_attitude": Quaternion((0, 0.7071068, 0, 0.7071068)),
                "expected_difference": Quaternion((0, 0.9255077, 0, 0.3787288))
            }
        ]

        test_count = 0
        for test in test_cases:
            test: Mapping[str, Quaternion]
            with self.subTest(
            msg=
            f"current attitude: {test['current_attitude']} -> set attitude {test['set_attitude']}"
            ):
                # the current attitude is at -90 deg from neutral or 0 deg pitch
                self.gimbal_gremsy._current_attitude = test["current_attitude"]
                # set the pitch to +90 deg from neutral or 0 deg pitch
                self.gimbal_gremsy.set_attitude(test["set_attitude"])

                expected_queue_call_arg: _SetAttitude = (
                    self.gimbal_gremsy._command_queue.put.call_args[0][0]
                )

                self.assertAlmostEqual(
                    expected_queue_call_arg.attitude[0], test["expected_difference"][0], 4
                )
                self.assertAlmostEqual(
                    expected_queue_call_arg.attitude[1], test["expected_difference"][1], 4
                )
                self.assertAlmostEqual(
                    expected_queue_call_arg.attitude[2], test["expected_difference"][2], 4
                )
                self.assertAlmostEqual(
                    expected_queue_call_arg.attitude[3], test["expected_difference"][3], 4
                )
                test_count += 1

        # 1 + test_count compensates for the single _command_queue.put in GimbalGremsy __init__
        self.assertEqual(self.gimbal_gremsy._command_queue.put.call_count, 1 + test_count)


    def test_set_attitude_when_current_attitude_none(self):
        self.gimbal_gremsy._do_mount_configure_status_complete = True
        # the current attitude is set to None during initialization receiving an attitude
        self.gimbal_gremsy._current_attitude = None
        # set the pitch to +90 deg from neutral or 0 deg pitch
        self.gimbal_gremsy.set_attitude(Quaternion((0, 0.7071, 0, 0.7071)))

        expected_queue_call_arg: _SetAttitude = (
            self.gimbal_gremsy._command_queue.put.call_args[0][0]
        )

        self.assertAlmostEqual(expected_queue_call_arg.attitude[0], 0, 4)
        self.assertAlmostEqual(expected_queue_call_arg.attitude[1], .7071, 4)
        self.assertAlmostEqual(expected_queue_call_arg.attitude[2], 0, 4)
        self.assertAlmostEqual(expected_queue_call_arg.attitude[3], 0.7071, 4)


    def test_set_attitude_do_mount_configure_status_complete_false(self):
        self.gimbal_gremsy._do_mount_configure_status_complete = False

        self.gimbal_gremsy.set_attitude(Quaternion((0, 0.7071, 0, 0.7071)))

        self.gimbal_gremsy._do_mount_configure_status_lock.__enter__.assert_called_once_with()
        # start with check of index 1 because _MountConfigure already put once in GremsyGimbal init
        self.assertIsInstance(
            self.gimbal_gremsy._command_queue.put.call_args_list[1][0][0],
            _MountConfigure
        )
        self.assertIsInstance(
            self.gimbal_gremsy._command_queue.put.call_args_list[2][0][0],
            _SetAttitude
        )


    def test_set_neutral_attitude(self):
        self.gimbal_gremsy.set_neutral_attitude()

        expected_queue_call_arg: _ResetGimbal = (
            self.gimbal_gremsy._command_queue.put.call_args[0][0]
        )

        self.assertEqual(expected_queue_call_arg.command_type, _CmdType.RESET_GIMBAL)


    def test_receive_command_ack(self):
        mock_message = mavlink2.MAVLink_command_ack_message(
            command = mavlink2.MAV_CMD_DO_MOUNT_CONFIGURE,
            result = mavlink2.MAV_RESULT_ACCEPTED
        )
        self.gimbal_gremsy._receive_command_ack(msg=mock_message)

        self.gimbal_gremsy._do_mount_configure_status_lock.__enter__.assert_called_once_with()
        self.assertTrue(self.gimbal_gremsy._do_mount_configure_status_complete)


    def test_receive_raw_imu(self):
        mock_message = mavlink2.MAVLink_raw_imu_message(
            time_usec=0,
            xacc=0,
            yacc=0,
            zacc=0,
            xgyro=0,
            ygyro=0,
            zgyro=0,
            xmag=0,
            ymag=0,
            zmag=0
        )

        # currently this callback does nothing, so simply testing that it is callable
        self.gimbal_gremsy._receive_raw_imu(msg=mock_message)


    def test_receive_sys_status(self):
        uint16_max = 65535 # sometimed indicates unused mavlink field
        mock_message = mavlink2.MAVLink_sys_status_message(
            onboard_control_sensors_present=0, 
            onboard_control_sensors_enabled=0, 
            onboard_control_sensors_health=0, 
            load=0, 
            voltage_battery=uint16_max, 
            current_battery=-1, 
            battery_remaining=-1, 
            drop_rate_comm=0, 
            errors_comm=0, 
            errors_count1=0, 
            errors_count2=0, 
            errors_count3=0,
            errors_count4=0
        )

        # currently this callback does nothing, so simply testing that it is callable
        self.gimbal_gremsy._receive_sys_status(msg=mock_message)


    def test_receive_mount_orientation(self):
        mock_message = mavlink2.MAVLink_mount_orientation_message(
            time_boot_ms=5500,
            roll=15.235,
            pitch=32.672,
            yaw=0,
            yaw_absolute=math.nan
        )

        # currently this callback does nothing, so simply testing that it is callable
        self.gimbal_gremsy._receive_mount_orientation(msg=mock_message)

        self.gimbal_gremsy._current_attitude_lock.__enter__.assert_called_once_with()
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[0], 0.1272076, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[1], 0.2787875, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[2], 0.0372849, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[3], 0.9511601, 7)


    def test_receive_mount_orientation_invalid_axes_nan(self):
        mock_message = mavlink2.MAVLink_mount_orientation_message(
            time_boot_ms=5500,
            roll=math.nan,
            pitch=math.nan,
            yaw=math.nan,
            yaw_absolute=math.nan
        )

        # currently this callback does nothing, so simply testing that it is callable
        self.gimbal_gremsy._receive_mount_orientation(msg=mock_message)

        self.gimbal_gremsy._current_attitude_lock.__enter__.assert_called_once_with()
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[0], 0, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[1], 0, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[2], 0, 7)
        self.assertAlmostEqual(self.gimbal_gremsy._current_attitude[3], 1, 7)


    def test_quaternion_to_axis_angle(self):
        test_cases = [
            {
                "quaternion": Quaternion((0, 0.9238795, 0, 0.3826834)),
                "expected_x_angle": 0,
                "expected_y_angle": 135.0,
                "expected_z_angle": 0
            },
            {
                "quaternion": Quaternion((-0.0342057, 0.9235551, 0, 0.381937)),
                "expected_x_angle": -5.0,
                "expected_y_angle": 135.0,
                "expected_z_angle": 0
            },
            {
                "quaternion": Quaternion((0, 0, 0, 1)),
                "expected_x_angle": 0,
                "expected_y_angle": 0,
                "expected_z_angle": 0
            }
        ]

        for test in test_cases:
            with self.subTest(msg=f"Quaternion input: {test['quaternion']}"):
                x_roll, y_pitch, z_yaw = self.gimbal_gremsy._quaternion_to_axis_angle(
                    test["quaternion"]
                )

                self.assertAlmostEqual(x_roll, test["expected_x_angle"], 3)
                self.assertAlmostEqual(y_pitch, test["expected_y_angle"], 3)
                self.assertAlmostEqual(z_yaw, test["expected_z_angle"], 3)


    def test_gained_connection(self):
        mock_gimbal_status_lock = MagicMock()
        self.gimbal_gremsy._gimbal_status_lock = mock_gimbal_status_lock
        self.gimbal_gremsy._gained_connection()

        mock_gimbal_status_lock.__enter__.assert_called_once_with()
        self.assertEqual(
            self.gimbal_gremsy._gimbal_status,
            GimbalStatus.CONNECTED_BUT_NOT_IN_CONTROL
        )


    def test_lost_connection(self):
        mock_gimbal_status_lock = MagicMock()
        self.gimbal_gremsy._gimbal_status_lock = mock_gimbal_status_lock
        self.gimbal_gremsy._lost_connection()

        mock_gimbal_status_lock.__enter__.assert_called_once_with()
        self.assertEqual(
            self.gimbal_gremsy._gimbal_status,
            GimbalStatus.NOT_CONNECTED
        )

