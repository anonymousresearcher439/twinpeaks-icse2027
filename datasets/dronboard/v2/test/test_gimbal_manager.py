import math
from pymavlink.dialects.v20 import common as mavlink2
from queue import Queue
from threading import Event, Lock, Thread
import unittest
from unittest import skip
from unittest.mock import Mock, NonCallableMagicMock, NonCallableMock, patch

from dr_onboard_autonomy.models.gimbal import (
    GimbalAxes,
    GimbalStatus,
    Quaternion
)
from dr_onboard_autonomy.gimbal.gimbal_manager import (
    _CmdType,
    _GimbalManagerFlags,
    _ResetGimbal,
    _SetAttitude,
    _TakeControl,
    GimbalManager,
)
from dr_onboard_autonomy.mavlink import ComponentId, MavlinkNode, MavlinkSender, SystemId


class TestGimbalManager(unittest.TestCase):
    @patch("dr_onboard_autonomy.models.gimbal.Lock", spec=Lock)
    @patch("dr_onboard_autonomy.models.gimbal.Queue", spec=Queue)
    @patch("dr_onboard_autonomy.models.gimbal.Event", spec=Event)
    @patch("dr_onboard_autonomy.models.gimbal.Thread", spec=Thread)
    def setUp(
        self,
        mock_Thread: NonCallableMagicMock,
        mock_Event: NonCallableMagicMock,
        mock_Queue: NonCallableMagicMock,
        mock_Lock: NonCallableMagicMock
    ):
        self.mock_mavlink_sender = NonCallableMock(spec=MavlinkSender)
        self.mock_controller_node = MavlinkNode(SystemId(1), ComponentId(1))
        self.mock_gimbal_manager_node = MavlinkNode(SystemId(1), ComponentId(3))
        self.gimbal_mavros = GimbalManager(
            mavlink_sender=self.mock_mavlink_sender,
            controller=self.mock_controller_node,
            gimbal_axes=GimbalAxes.PITCH | GimbalAxes.ROLL,
            gimbal_manager=self.mock_gimbal_manager_node,
            gimbal_device_id=ComponentId(11)
        )


    @patch("dr_onboard_autonomy.models.gimbal.Queue", spec=Queue)
    @patch("dr_onboard_autonomy.models.gimbal.Event", spec=Event)
    @patch("dr_onboard_autonomy.models.gimbal.Thread", spec=Thread)
    def test_init(
        self,
        mock_Thread: NonCallableMagicMock,
        mock_Event: NonCallableMagicMock,
        mock_Queue: NonCallableMagicMock
    ):
        mock_mavlink_sender = NonCallableMock(spec=MavlinkSender)
        controller_node = MavlinkNode(SystemId(1), ComponentId(1))
        gimbal_manager_node = MavlinkNode(SystemId(1), ComponentId(3))
        gimbal_mavros = GimbalManager(
            mavlink_sender=mock_mavlink_sender,
            controller=controller_node,
            gimbal_axes=GimbalAxes.PITCH | GimbalAxes.ROLL,
            gimbal_manager=gimbal_manager_node,
            gimbal_device_id=ComponentId(11)
        )

        self.assertEqual(gimbal_mavros._mavlink_sender, mock_mavlink_sender)
        
        # test take control of gimbal
        expected_queue_call_arg: _TakeControl = mock_Queue.return_value.put.call_args[0][0]
        self.assertEqual(expected_queue_call_arg.command_type, _CmdType.TAKE_CONTROL)
        self.assertEqual(expected_queue_call_arg.gimbal_manager, gimbal_manager_node)
        self.assertEqual(expected_queue_call_arg.gimbal_device_id, ComponentId(11))
        self.assertEqual(expected_queue_call_arg.primary_controller, controller_node)
        self.assertEqual(expected_queue_call_arg.secondary_controller.component_id, ComponentId(-1))
        self.assertEqual(expected_queue_call_arg.secondary_controller.system_id, SystemId(-1))

    
    def test_send_take_control(self):
        msg_take_control=_TakeControl(
            gimbal_device_id=ComponentId(15),
            gimbal_manager=self.mock_gimbal_manager_node,
            primary_controller=self.mock_controller_node,
            secondary_controller=MavlinkNode(SystemId(-1), ComponentId(-1)),
        )
        
        self.gimbal_mavros._send_take_control(msg=msg_take_control)
        self.mock_mavlink_sender.send.assert_called_once_with(
                mavlink2.MAVLink_command_long_message(
                target_system=self.mock_gimbal_manager_node.system_id,
                target_component=self.mock_gimbal_manager_node.component_id,
                command=mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                confirmation=0,
                param1=self.mock_controller_node.system_id,
                param2=self.mock_controller_node.component_id,
                param3=SystemId(-1),
                param4=ComponentId(-1),
                param5=0,
                param6=0,
                param7=ComponentId(15)
            )
        )

    
    def test_process_command_queue(self):
        mock_command = _TakeControl(
            gimbal_device_id=ComponentId(20),
            gimbal_manager=self.mock_gimbal_manager_node,
            primary_controller=self.mock_controller_node,
            secondary_controller=MavlinkNode(SystemId(-1), ComponentId(-1)),
        )

        self.gimbal_mavros._command_queue.get.return_value = mock_command
        self.gimbal_mavros._process_command_queue()

        mavlink_command: mavlink2.MAVLink_command_long_message = (
            self.mock_mavlink_sender.send.call_args[0][0]
        )

        self.assertEqual(mavlink_command.param7, mock_command.gimbal_device_id)


    def test_reaquire_control(self):
        self.gimbal_mavros._put_take_control = Mock()

        self.gimbal_mavros.reaquire_control()

        self.gimbal_mavros._put_take_control.assert_called_once_with(
            controller=self.gimbal_mavros._controller,
            gimbal_device_id=self.gimbal_mavros._gimbal_device_id,
            gimbal_manager=self.gimbal_mavros._gimbal_manager
        )


    def test_set_attitude(self):
        self.gimbal_mavros.set_attitude(Quaternion([0, 0.7071, 0, 0.7071]))
        
        expected_queue_call_arg: _SetAttitude = self.gimbal_mavros._command_queue.put.call_args[0][0]

        self.assertEqual(expected_queue_call_arg.attitude, (0.7071, 0, 0.7071, 0))
        self.assertEqual(expected_queue_call_arg.gimbal_manager, self.gimbal_mavros._gimbal_manager)
        self.assertEqual(expected_queue_call_arg.gimbal_device_id, self.gimbal_mavros._gimbal_device_id)


    def test_send_set_attitude(self):
        set_attitude_message = _SetAttitude(
            attitude=[0.7071, 0, 0.7071, 0],
            gimbal_device_id=ComponentId(3),
            gimbal_manager=MavlinkNode(SystemId(1), ComponentId(5))
        )
        
        self.gimbal_mavros._send_set_attitude(msg=set_attitude_message)

        mav_set_attitude_cmd = self.mock_mavlink_sender.send.call_args[0][0]
        
        self.assertIsInstance(
            mav_set_attitude_cmd,
            mavlink2.MAVLink_gimbal_manager_set_attitude_message
        )

        self.assertEqual(
            mav_set_attitude_cmd.target_system,
            set_attitude_message.gimbal_manager.system_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.target_component,
            set_attitude_message.gimbal_manager.component_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.flags,
            _GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NOTHING.value
        )
        self.assertEqual(
            mav_set_attitude_cmd.gimbal_device_id,
            set_attitude_message.gimbal_device_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.q,
            set_attitude_message.attitude
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_x)
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_y)
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_z)
        )


        self.gimbal_mavros._current_attitude_lock.__enter__.assert_called_with()

        self.assertEqual(self.gimbal_mavros._current_attitude, Quaternion((0, 0.7071, 0, 0.7071)))


    @skip("TODO: implement in the future if Mavros starts publishing topics related to gimbals")
    def test_receive_message_callback(self):
        pass


    def test_set_neutral_attitude(self):
        self.gimbal_mavros.set_neutral_attitude()

        reset_gimbal_msg: _ResetGimbal = self.gimbal_mavros._command_queue.put.call_args[0][0]

        self.assertEqual(reset_gimbal_msg.gimbal_manager, self.gimbal_mavros._gimbal_manager)
        self.assertEqual(reset_gimbal_msg.gimbal_device_id, self.gimbal_mavros._gimbal_device_id)


    def test_send_set_neutral_attitude(self):
        set_neutral_attitude_message = _ResetGimbal(
            gimbal_device_id=ComponentId(3),
            gimbal_manager=MavlinkNode(SystemId(1), ComponentId(5))
        )
        
        self.gimbal_mavros._send_set_neutral_attitude(msg=set_neutral_attitude_message)

        mav_set_attitude_cmd = self.mock_mavlink_sender.send.call_args[0][0]
        
        self.assertIsInstance(
            mav_set_attitude_cmd,
            mavlink2.MAVLink_gimbal_manager_set_attitude_message
        )

        self.assertEqual(
            mav_set_attitude_cmd.target_system,
            set_neutral_attitude_message.gimbal_manager.system_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.target_component,
            set_neutral_attitude_message.gimbal_manager.component_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.flags,
            _GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NOTHING.value
        )
        self.assertEqual(
            mav_set_attitude_cmd.gimbal_device_id,
            set_neutral_attitude_message.gimbal_device_id
        )
        self.assertEqual(
            mav_set_attitude_cmd.q,
            (math.nan, math.nan, math.nan, math.nan)
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_x)
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_y)
        )
        self.assertTrue(
            math.isnan(mav_set_attitude_cmd.angular_velocity_z)
        )


        self.gimbal_mavros._current_attitude_lock.__enter__.assert_called_with()

        self.assertEqual(self.gimbal_mavros._current_attitude, Quaternion((0, 0, 0, 1)))


    def test_get_gimbal_status(self):
        gimbal_status: GimbalStatus = self.gimbal_mavros.get_gimbal_status()

        self.assertEqual(gimbal_status, GimbalStatus.UNKNOWN)


    def test_get_gimbal_axes(self):
        self.assertEqual(
            self.gimbal_mavros.get_gimbal_axes(),
            GimbalAxes.PITCH | GimbalAxes.ROLL
        )