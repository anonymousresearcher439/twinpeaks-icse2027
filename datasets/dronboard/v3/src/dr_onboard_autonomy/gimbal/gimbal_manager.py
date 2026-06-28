from collections.abc import Mapping
from dataclasses import dataclass
import enum
import math
import time
from typing import Callable, Tuple

from pymavlink.dialects.v20 import common as mavlink2

from dr_onboard_autonomy.models import Quaternion
from dr_onboard_autonomy.models.gimbal import Gimbal, GimbalAxes, GimbalStatus
from dr_onboard_autonomy.mavlink import ComponentId, MavlinkNode, MavlinkSender, SystemId


# The set of mavlink messages we want to receive as input
# https://mavlink.io/en/services/gimbal_v2.html#messagecommandenum-summary
MAVLINK_GIMBAL_MSG_IDS = {
    # Gimbal manager messages
    mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
    mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
    # mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE,
    # mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW,

    # Gimbal device messages
    mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
    #mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE,
    #mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
}


class _CmdType(enum.Enum):
    EXIT = enum.auto()
    FIND_GIMBALS = enum.auto()
    GET_GIMBALS = enum.auto()
    RECV_MAVLINK = enum.auto()
    SET_ATTITUDE  = enum.auto()
    RESET_GIMBAL = enum.auto()
    TAKE_CONTROL = enum.auto()


# GIMBAL_MANAGER_FLAGS
class _GimbalManagerFlags(enum.Flag):
    GIMBAL_MANAGER_FLAGS_NOTHING    = 0
    GIMBAL_MANAGER_FLAGS_RETRACT    = 1 # Based on GIMBAL_DEVICE_FLAGS_RETRACT
    GIMBAL_MANAGER_FLAGS_NEUTRAL    = 2 # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK  = 4 # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8 # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
    GIMBAL_MANAGER_FLAGS_YAW_LOCK   = 16 # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK


@dataclass
class _ResetGimbal:
    gimbal_device_id: ComponentId
    gimbal_manager: MavlinkNode

    command_type: _CmdType = _CmdType.RESET_GIMBAL


@dataclass
class _SetAttitude:
    attitude: Tuple[float, float, float, float] # w, x, y, z
    gimbal_device_id: ComponentId
    gimbal_manager: MavlinkNode

    command_type: _CmdType = _CmdType.SET_ATTITUDE


@dataclass
class _TakeControl:
    gimbal_device_id: ComponentId
    gimbal_manager: MavlinkNode
    primary_controller: MavlinkNode
    secondary_controller: MavlinkNode

    command_type: _CmdType = _CmdType.TAKE_CONTROL


@dataclass
class _Exit:
    command_type: _CmdType = _CmdType.EXIT


class GimbalManager(Gimbal):
    def __init__(
            self,
            mavlink_sender: MavlinkSender,
            controller: MavlinkNode,
            gimbal_axes: GimbalAxes,
            gimbal_manager: MavlinkNode,
            gimbal_device_id: ComponentId=ComponentId(0),
        ):
        self._mavlink_sender: MavlinkSender = mavlink_sender
        self._controller: MavlinkNode = controller
        self._gimbal_manager: MavlinkNode = gimbal_manager
        self._gimbal_device_id: ComponentId = gimbal_device_id

        # implement line below if mavros publishes gimbal messages in the future
        # self._mavlink2 = mavlink2.MAVLink()
        self._command_to_sender_map: Mapping[_CmdType, Callable] = {
            _CmdType.TAKE_CONTROL: self._send_take_control,
            _CmdType.SET_ATTITUDE: self._send_set_attitude,
            _CmdType.RESET_GIMBAL: self._send_set_neutral_attitude,
            _CmdType.EXIT: self._cmd_exit,
        }

        super().__init__()
        self._gimbal_axes: GimbalAxes = gimbal_axes

        self._put_take_control(self._controller, self._gimbal_device_id, self._gimbal_manager)


    def _process_command_queue(self):
        msg_to_send = self._command_queue.get()
        self._command_to_sender_map[msg_to_send.command_type](msg_to_send)


    def _process_messages(self):
        self._exit_event.wait()


    def _put_take_control(
            self,
            controller: MavlinkNode,
            gimbal_device_id: ComponentId,
            gimbal_manager: MavlinkNode
        ):
            self._command_queue.put(_TakeControl(
                gimbal_manager=gimbal_manager,
                primary_controller=controller,
                # -1 means don't change the controller
                secondary_controller=MavlinkNode(SystemId(-1), ComponentId(-1)),
                gimbal_device_id=gimbal_device_id
            ))


    # bit of insight into encoding and decoding mavlink messages
    # https://james1345.medium.com/working-with-mavlink-messages-in-ros-without-mavros-88a055973fdf
    def receive_message_callback(self, message):
        pass
        # NOTE: implement in the future if Mavros starts publishing topics related to gimbals
        # http://wiki.ros.org/mavros#mavros.2FPlugins.Published_Topics-8
        # # convert ROS topic message type to mavlink bytes
        # msg_bytes: bytearray = rosmavlink.convert_to_bytes(message)
        # # decode mavlink bytes to get mavlink message as defined in pymavlink library
        # mavmsg: mavlink2.MAVLink_message = self._mavlink2.decode(msg_bytes)
        # # If we're receiving a message that we care about for to gimbal operations
        # if mavmsg.get_msgId() in MAVLINK_GIMBAL_MSG_IDS:
        #     self._received_messages_queue.put(mavmsg)


    def _send_set_attitude(self, msg: _SetAttitude):
        # based on the description of GIMBAL_DEVICE_FLAGS, the default coordinate reference frame
        # has pitch and roll in the global reference frame and yaw relative to the vehicle heading -
        # essentially the quaternion is relative to a global reference frame with a rotation around
        # the z-axis so the x-axis aligns with the front of the vehicle
        self._mavlink_sender.send(
            mavlink2.MAVLink_gimbal_manager_set_attitude_message(
                target_system=msg.gimbal_manager.system_id,
                target_component=msg.gimbal_manager.component_id,
                flags=_GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NOTHING.value,
                gimbal_device_id=msg.gimbal_device_id,
                q=msg.attitude,
                angular_velocity_x=math.nan,
                angular_velocity_y=math.nan,
                angular_velocity_z=math.nan
            )
        )

        with self._attitude_lock:
            self.attitude.add_attitude(time.time(), Quaternion(*msg.attitude[1:4], msg.attitude[0]))


    def _send_set_neutral_attitude(self, msg: _ResetGimbal):
        self._mavlink_sender.send(
            mavlink2.MAVLink_gimbal_manager_set_attitude_message(
                target_system=msg.gimbal_manager.system_id,
                target_component=msg.gimbal_manager.component_id,
                flags=_GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NOTHING.value,
                gimbal_device_id=msg.gimbal_device_id,
                q=(math.nan, math.nan, math.nan, math.nan),
                angular_velocity_x=math.nan,
                angular_velocity_y=math.nan,
                angular_velocity_z=math.nan
            )
        )

        with self._attitude_lock:
            self.attitude.add_attitude(time.time(), Quaternion(0, 0, 0, 1))


    def _send_take_control(self, msg: _TakeControl):
        # this mavlink message is documented here:
        # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        self._mavlink_sender.send(mavlink2.MAVLink_command_long_message(
                target_system=msg.gimbal_manager.system_id,
                target_component=msg.gimbal_manager.component_id,
                command=mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                confirmation=0,
                param1=msg.primary_controller.system_id,
                param2=msg.primary_controller.component_id,
                param3=msg.secondary_controller.system_id,
                param4=msg.secondary_controller.component_id,
                param5=0,
                param6=0,
                param7=msg.gimbal_device_id
            )
        )


    def get_gimbal_status(self) -> GimbalStatus:
        # returns unknown because no gimbal related messages are received through mavros
        return GimbalStatus.UNKNOWN


    def get_gimbal_axes(self) -> GimbalAxes:
        return self._gimbal_axes


    def reaquire_control(self):
        self._put_take_control(
            controller=self._controller,
            gimbal_device_id=self._gimbal_device_id,
            gimbal_manager=self._gimbal_manager
        )


    def set_attitude(self, attitude: Quaternion):
        mavlink_q = (attitude.w, attitude.x, attitude.y, attitude.z)

        self._command_queue.put(_SetAttitude(
            attitude=mavlink_q,
            gimbal_manager=self._gimbal_manager,
            gimbal_device_id = self._gimbal_device_id,
        ))


    def set_neutral_attitude(self):
        self._command_queue.put(_ResetGimbal(
            gimbal_manager=self._gimbal_manager,
            gimbal_device_id=self._gimbal_device_id
        ))
    
    def _on_exit(self):
        self._command_queue.put(_Exit())
        super()._on_exit()
    
    def _cmd_exit(self, _):
        # rospy.logininfo("Exiting GimbalManager")
        pass
