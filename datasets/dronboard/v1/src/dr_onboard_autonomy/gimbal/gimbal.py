import enum
import math

from dataclasses import dataclass, field
from math import nan
from queue import Empty, Queue
from threading import Thread, Lock
from time import time, sleep
from typing import Callable, Dict, List, MutableMapping, NoReturn, Optional, Tuple, Union, NewType

import mavros.mavlink
import rospy

from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2
from tf.transformations import quaternion_about_axis

from dr_onboard_autonomy.mavlink import MavlinkSender
from dr_onboard_autonomy.frames import AttitudeData

"""
>>import rospy
>>rospy.init_node("gimbal_example")
>>
>>from dr_onboard_autonomy import MAVROSDrone
>>from dr_onboard_autonomy.gimbal import MavlinkNode, Quaternion
>>from tf.transformations import quaternion_about_axis
>>import math
>>
>>drone = MAVROSDrone("birdy0")
>>drone.start()
>>gimbal_manager = MavlinkNode(1, 1)
>>
>>drone.gimbal.take_control(gimbal_manager)
>>
>>q = quaternion_about_axis(math.radians(90), (0,0,1))
>>drone.gimbal.set_attitude(q, gimbal_manager)
>>drone.gimbal.reset_attitude(gimbal_manager)
"""


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


# This is a python translation of GIMBAL_MANAGER_CAP_FLAGS
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_CAP_FLAGS
class _GimbalManagerCapability(enum.Flag):
    GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT               = 1 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL               = 2 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS             = 4 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW           = 8 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK             = 16 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS            = 32 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW          = 64 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK            = 128 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS              = 256 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW            = 512 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK              = 1024 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW     = 2048 # Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL  = 65536 # Gimbal manager supports to point to a local position.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 # Gimbal manager supports to point to a global latitude, longitude, altitude position.


SystemId = NewType('SystemId', int)


ComponentId = NewType('ComponentId', int)


@dataclass(order=True, frozen=True)
class MavlinkNode:
    system_id: SystemId
    component_id: ComponentId

# quaternions are represented x, y, z, w (to match what the tf library does)
Quaternion = NewType('Quaternion', Tuple[float, float, float, float])


@dataclass
# This class is based on the GIMBAL_MANAGER_INFORMATION message
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
class GimbalManager:
    system_id: SystemId
    component_id: ComponentId
    capability_flags: _GimbalManagerCapability # Bitmap of gimbal capability flags.
    gimbal_device_id: ComponentId # Gimbal device ID that this gimbal manager is responsible for.
    roll_min: float # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left) in radians
    roll_max: float # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left) in radians
    pitch_min: float # Minimum pitch angle (positive: up, negative: down) in radians
    pitch_max: float # Maximum pitch angle (positive: up, negative: down) in radians
    yaw_min: float # Minimum yaw angle (positive: to the right, negative: to the left) in radians
    yaw_max: float # Maximum yaw angle (positive: to the right, negative: to the left) in radians

    @property
    def mavlink_node(self) -> MavlinkNode:
        return MavlinkNode(self.system_id, self.component_id)


# GIMBAL_MANAGER_FLAGS
class _GimbalManagerFlags(enum.Flag):
    GIMBAL_MANAGER_FLAGS_NOTHING    = 0
    GIMBAL_MANAGER_FLAGS_RETRACT    = 1 # Based on GIMBAL_DEVICE_FLAGS_RETRACT
    GIMBAL_MANAGER_FLAGS_NEUTRAL    = 2 # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK  = 4 # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8 # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
    GIMBAL_MANAGER_FLAGS_YAW_LOCK   = 16 # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK


# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
@dataclass
class _GimbalManagerStatus:
    system_id: SystemId
    component_id: ComponentId
    flags: _GimbalManagerFlags # High level gimbal manager flags currently applied.
    gimbal_device_id: ComponentId # Gimbal device ID that this gimbal manager is responsible for.
    primary_control_sysid: SystemId # System ID of MAVLink component with primary control, 0 for none.
    primary_control_compid: ComponentId # Component ID of MAVLink component with primary control, 0 for none.
    secondary_control_sysid: SystemId # System ID of MAVLink component with secondary control, 0 for none.
    secondary_control_compid: ComponentId # Component ID of MAVLink component with secondary control, 0 for none.

    @property
    def primary_controller(self) -> MavlinkNode:
        return MavlinkNode(self.primary_control_sysid, self.primary_control_compid)

    @property
    def secondary_controller(self) -> MavlinkNode:
        return MavlinkNode(self.secondary_control_sysid, self.secondary_control_compid)


# This is a python translation of GIMBAL_DEVICE_FLAGS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_FLAGS
class _GimbalDeviceFlags(enum.Flag):
    GIMBAL_DEVICE_FLAGS_RETRACT    = 1 # Set to retracted safe position (no stabilization), takes presedence over all other flags.
    GIMBAL_DEVICE_FLAGS_NEUTRAL    = 2 # Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT.
    GIMBAL_DEVICE_FLAGS_ROLL_LOCK  = 4 # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8 # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
    GIMBAL_DEVICE_FLAGS_YAW_LOCK   = 16 # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).


# this is a python translation of GIMBAL_DEVICE_ERROR_FLAGS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ERROR_FLAGS
class _GimbalDeviceErrorFlags(enum.Flag):
    NOTHING                                       = 0 # for no failure
    GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT       = 1 # Gimbal device is limited by hardware roll limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT      = 2 # Gimbal device is limited by hardware pitch limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT        = 4 # Gimbal device is limited by hardware yaw limit.
    GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR       = 8 # There is an error with the gimbal encoders.
    GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR         = 16 # There is an error with the gimbal power source.
    GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR         = 32 # There is an error with the gimbal motor's.
    GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR      = 64 # There is an error with the gimbal's software.
    GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR         = 128 # There is an error with the gimbal's communication.
    GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256 # Gimbal is currently calibrating.


@dataclass
class _GimbalStatus:
    system_id: SystemId
    component_id: ComponentId
    flags: _GimbalDeviceFlags
    q: Quaternion
    angular_velocity_x: float # radians per second
    angular_velocity_y: float # radians per second
    angular_velocity_z: float # radians per second
    failure_flags: _GimbalDeviceErrorFlags


class _CmdType(enum.Enum):
    EXIT = enum.auto()
    FIND_GIMBALS = enum.auto()
    GET_GIMBALS = enum.auto()
    RECV_MAVLINK = enum.auto()
    SET_ATTITUDE  = enum.auto()
    RESET_GIMBAL = enum.auto()
    TAKE_CONTROL = enum.auto()


@dataclass
class _Command:
    command_type: _CmdType

    # used for SET_ATTITUDE
    quaternion: Quaternion = None
    gimbal_manager: MavlinkNode = None

    # used for receiving mavlink
    mavlink_message: mavlink2.MAVLink_message = None

    # used for FIND_GIMBALS and GET_GIMBALS
    return_channel: Queue = field(default_factory=Queue)

    # used for TAKE_CONTROL command
    primary_controller: MavlinkNode = None
    secondary_controller: MavlinkNode = None
    gimbal_device_id: ComponentId = ComponentId(0)


_InternalCmdMethod = Callable[[_Command], None]


class Gimbal:

    def __init__(self, mav_sender: MavlinkSender):
        self.mav_sender = mav_sender
        self.cmd_queue = Queue()
        self.gimbal_managers: MutableMapping[MavlinkNode, GimbalManager] = dict()
        self.gimbal_manager_status: MutableMapping[MavlinkNode, _GimbalManagerStatus] = dict()
        self.gimbal_status: MutableMapping[MavlinkNode, _GimbalStatus] = dict()

        self.attitude_data = AttitudeData()

        self.worker_thread = Thread(target=self._run)
        self.lock = Lock()
        self._attitude: Optional[Quaternion] = None

        self._cmd_method: Dict[_CmdType, _InternalCmdMethod] = {
            _CmdType.FIND_GIMBALS: self._find_gimbals,
            _CmdType.GET_GIMBALS: self._get_gimbals,
            _CmdType.RECV_MAVLINK: self._recv_mavlink,
            _CmdType.SET_ATTITUDE: self._set_attitude,
            _CmdType.TAKE_CONTROL: self._take_control,
            _CmdType.RESET_GIMBAL: self._reset_gimbal,
        }

        self._recv_mavlink_method = {
            mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: self._recv_gimbal_manager_information,
            mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: self._recv_gimbal_manager_status,
            mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: self._recv_gimbal_device_attitude_status,
        }

    def start(self):
        self.worker_thread.start()
    
    def stop(self, join=True):
        if not self.worker_thread.is_alive():
            return
        cmd = _Command(
            command_type=_CmdType.EXIT
        )
        self.cmd_queue.put(cmd)
        if join:
            self.worker_thread.join()
    
    @property
    # TODO fix the case where this gimbal object is controlling more than one gimbal...
    def attitude(self) -> Optional[Quaternion]:
        """Return the last attitude that was set to a gimbal (if any)"""
        with self.lock:
            return self._attitude

    def find_gimbals(self, wait_time=5.0) -> List[GimbalManager]:
        """This method broadcasts a mavlink message that requests all gimbal managers to respond with their details.
        
        The wait_time parameter is how long this method will wait responses in seconds. Any responses we receive after this amount of time won't be included in the return value.
        """
        cmd = _Command(command_type=_CmdType.FIND_GIMBALS)
        self.cmd_queue.put(cmd)

        def response_func():
            sleep(wait_time)
            gimbals = self.get_gimbals()
            cmd.return_channel.put(gimbals)
        
        response_thread = Thread(target=response_func, daemon=True)
        response_thread.start()
        return cmd.return_channel.get(timeout=wait_time+1.0)

    def get_gimbals(self) -> List[GimbalManager]:
        """This method returns the list of gimbal managers we already know about 
        """
        cmd = _Command(
            command_type=_CmdType.GET_GIMBALS
        )
        self.cmd_queue.put(cmd)
        return cmd.return_channel.get(timeout=1.0)

    def take_control(self,
                     gimbal_manager: Union[MavlinkNode, GimbalManager],
                     gimbal_id: ComponentId = 0,
                     primary_controller: MavlinkNode = None,
                     secondary_controller: MavlinkNode = None):
        """Tell the gimbal manager which mavlink node has primary control and which has secondary control.

        If primary_controller is set to None, then this object takes primary control.
        If secondary_controller is None, then don't change the secondary controller 
        """

        if type(gimbal_manager) == GimbalManager:
            gimbal_manager = gimbal_manager.mavlink_node

        if primary_controller is None:
            # make this the primary controller
            primary_controller = MavlinkNode(self.mav_sender.system_id, self.mav_sender.component_id)

        if secondary_controller is None:
            # -1 means don't change the controller
            secondary_controller = MavlinkNode(-1, -1)

        cmd = _Command(gimbal_manager=gimbal_manager,
                       command_type=_CmdType.TAKE_CONTROL,
                       primary_controller=primary_controller,
                       secondary_controller=secondary_controller,
                       gimbal_device_id=gimbal_id)
        self.cmd_queue.put(cmd)
    
    def set_attitude(self, q: Quaternion, gimbal_manager: Union[MavlinkNode, GimbalManager], gimbal_id: ComponentId = 0):
        """Set the attitude of the gimbal. 
        
        The quaternion parameter is specified using the convention from TF (X, Y, Z, W).
        The quaternion applies rotation in a forward right down coordinate system.

        The gimbal manager and gimbal device specify which gimbal you want to control. you need to
        take control of the gimbal manager before using this method or else the gimbal manager will
        ignore the command. You can use the `take_control` method to set this object as a controller.
        """
        # MAVLink expects w, x, y, z order but our Quaternion follows the x, y, z, w convention from the tf lib
        q = q[3], *q[:3]
        if type(gimbal_manager) == GimbalManager:
            gimbal_manager = gimbal_manager.mavlink_node
        cmd = _Command(
            command_type=_CmdType.SET_ATTITUDE,
            quaternion=q,
            gimbal_manager=gimbal_manager,
            gimbal_device_id = gimbal_id,
        )
        self.cmd_queue.put(cmd)
    
    def reset_attitude(self, gimbal_manager: Union[MavlinkNode, GimbalManager], gimbal_id: ComponentId = 0):
        if type(gimbal_manager) == GimbalManager:
            gimbal_manager = gimbal_manager.mavlink_node
        cmd = _Command(
            command_type=_CmdType.RESET_GIMBAL,
            gimbal_device_id = gimbal_id,
            gimbal_manager=gimbal_manager,
        )
        self.cmd_queue.put(cmd)

    def _run(self):
        def on_exit():
            self.stop(join=False)
        rospy.on_shutdown(on_exit)
        while not rospy.is_shutdown():
            cmd: _Command = self.cmd_queue.get()
            if cmd.command_type == _CmdType.EXIT:
                return

            method = self._cmd_method[cmd.command_type]
            method(cmd)

    def _mavlink_sub_callback(self, msg):
        msg_bytes: bytearray = mavros.mavlink.convert_to_bytes(msg)
        mavmsg = self._mav2.decode(msg_bytes)
        # If we're receiving a message that we care about for to gimbal operations
        if mavmsg.get_msgId() in MAVLINK_GIMBAL_MSG_IDS:
            self.cmd_queue.put(mavmsg)

    def _find_gimbals(self, cmd: _Command):
        # The protocol for "discovery of gimbal managers" is documented here:
        # https://mavlink.io/en/services/gimbal_v2.html#discovery-of-gimbal-manager
        # In summary, we need to broadcast a command that tells all the gimbal managers we want
        # information. The gimbal managers respond so we can know their component IDs and
        # capabilities.

        # To find the gimbal manager We need to create a COMMAND_LONG message
        # https://mavlink.io/en/messages/common.html#COMMAND_LONG

        # And our COMMAND_LONG needs to specify MAV_CMD_REQUEST_MESSAGE as the command
        # https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE

        ADDRESS_OF_REQUESTOR = 1 # constant for param7 of MAV_CMD_REQUEST_MESSAGE
        request_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1, # broadcast to all systems
            target_component=1, # broadcast to all components
            command=mavlink2.MAV_CMD_REQUEST_MESSAGE, # this command is a request message
            confirmation=0,
            param1=mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION, # the message we request
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=ADDRESS_OF_REQUESTOR # the Response Target
        )
        self.mav_sender.send(request_cmd)
    
    def _get_gimbals(self, cmd: _Command):
        result = list(self.gimbal_managers.values())
        cmd.return_channel.put(result)

    def _recv_mavlink(self, mavmsg: _Command):
        mavmsg: mavlink2.MAVLink_message = mavmsg.mavlink_message
        msg_id = mavmsg.get_msgId()
        recv_method = self._recv_mavlink_method[msg_id]
        recv_method(mavmsg)

    def _recv_gimbal_manager_information(self, msg: mavlink2.MAVLink_gimbal_manager_information_message):
        mavnode = MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        gimbal_manager = GimbalManager(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            capability_flags=_GimbalManagerCapability(msg.cap_flags),
            gimbal_device_id=msg.gimbal_device_id,
            roll_min=msg.roll_min,
            roll_max=msg.roll_max,
            pitch_min=msg.pitch_min,
            pitch_max=msg.pitch_max,
            yaw_min=msg.yaw_min,
            yaw_max=msg.yaw_max,
        )
        self.gimbal_managers[mavnode] = gimbal_manager

    def _recv_gimbal_manager_status(self, msg: mavlink2.MAVLink_gimbal_manager_status_message):
        mavnode = MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        manager_status = _GimbalManagerStatus(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            flags=_GimbalManagerFlags(msg.flags),
            gimbal_device_id=msg.gimbal_device_id,
            primary_control_sysid=msg.primary_control_sysid,
            primary_control_compid=msg.primary_control_compid,
            secondary_control_sysid=msg.secondary_control_sysid,
            secondary_control_compid=msg.secondary_control_compid,
        )
        self.gimbal_manager_status[mavnode] = manager_status

    def _recv_gimbal_device_attitude_status(self, msg: mavlink2.MAVLink_gimbal_device_attitude_status_message):
        mavnode = MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        gimbal_status = _GimbalStatus(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            flags=_GimbalDeviceFlags(msg.flags),
            q=(msg.q[0], msg.q[1], msg.q[2], msg.q[3]),
            angular_velocity_x=msg.angular_velocity_x,
            angular_velocity_y=msg.angular_velocity_y,
            angular_velocity_z=msg.angular_velocity_z,
            failure_flags=_GimbalDeviceErrorFlags(msg.failure_flags)
        )
        self.gimbal_status[mavnode] = gimbal_status

    def _take_control(self, msg: _Command):
        manager = msg.gimbal_manager
        primary = msg.primary_controller
        secondary = msg.secondary_controller

        # this mavlink message is documented here:
        # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        mav_cmd = mavlink2.MAVLink_command_long_message(
            target_system=manager.system_id,
            target_component=manager.component_id,
            command=mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            confirmation=0,
            param1=primary.system_id,
            param2=primary.component_id,
            param3=secondary.system_id,
            param4=secondary.component_id,
            param5=0,
            param6=0,
            param7=msg.gimbal_device_id
        )
        self.mav_sender.send(mav_cmd)


    def _set_attitude(self, msg: _Command):
        mav_cmd = mavlink2.MAVLink_gimbal_manager_set_attitude_message(
            target_system=msg.gimbal_manager.system_id,
            target_component=msg.gimbal_manager.component_id,
            flags=_GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NOTHING.value,
            gimbal_device_id=msg.gimbal_device_id,
            q=msg.quaternion,
            angular_velocity_x=math.nan,
            angular_velocity_y=math.nan,
            angular_velocity_z=math.nan,
        )
        self.mav_sender.send(mav_cmd)
        with self.lock:
            self._attitude = *msg.quaternion[1:4], msg.quaternion[0]
    
    def _reset_gimbal(self, cmd: _Command):
        mav_cmd = mavlink2.MAVLink_gimbal_manager_set_attitude_message(
            target_system=cmd.gimbal_manager.system_id,
            target_component=cmd.gimbal_manager.component_id,
            flags=_GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NEUTRAL.value,
            gimbal_device_id=cmd.gimbal_device_id,
            q=[math.nan, math.nan, math.nan, math.nan],
            angular_velocity_x=math.nan,
            angular_velocity_y=math.nan,
            angular_velocity_z=math.nan,
        )
        self.mav_sender.send(mav_cmd)
