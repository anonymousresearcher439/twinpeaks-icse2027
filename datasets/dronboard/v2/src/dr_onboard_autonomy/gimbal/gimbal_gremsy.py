import array
from collections.abc import Mapping
from dataclasses import dataclass
import enum
import math
from queue import Empty, Queue
from threading import RLock
from typing import Callable, Optional, Tuple, Union

from pymavlink.dialects.v20 import common as mavlink2
from pymavlink.dialects.v20 import ardupilotmega as mavlink2_mega
import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply

from dr_onboard_autonomy.models.gimbal import Gimbal, GimbalAxes, GimbalStatus, Quaternion
from dr_onboard_autonomy.mavlink import ComponentId, MavlinkNode, MavlinkRouterTCP, SystemId


class _CmdType(enum.Enum):
    MOUNT_CONFIGURE = enum.auto()
    RESET_GIMBAL = enum.auto()
    SET_ATTITUDE  = enum.auto()


class _GimbalAxisInputMode(enum.Enum):
    """Mavlink gimbal axis modes
    """
    CTRL_ANGLE_BODY_FRAME = 0 # body frame
    CTRL_ANGULAR_RATE = 1 # ground frame
    CTRL_ANGLE_ABSOLUTE_FRAME = 2 # ground frame


class _CmdProcessMode(enum.Enum):
    """Indicate how a command should be processed depending on the gimbal status
    """
    ALWAYS = 0 # process the command regardless of the gimbal status
    WAIT_FOR_READY = 1
    SKIP_IF_NOT_READY = 2


@dataclass
class _MountConfigure:
    pitch_mode: _GimbalAxisInputMode = _GimbalAxisInputMode.CTRL_ANGLE_ABSOLUTE_FRAME
    roll_mode: _GimbalAxisInputMode = _GimbalAxisInputMode.CTRL_ANGLE_ABSOLUTE_FRAME
    yaw_mode: _GimbalAxisInputMode = _GimbalAxisInputMode.CTRL_ANGLE_BODY_FRAME

    command_type: _CmdType = _CmdType.MOUNT_CONFIGURE
    command_process_mode: _CmdProcessMode = _CmdProcessMode.ALWAYS

@dataclass
class _ResetGimbal:
    command_type: _CmdType = _CmdType.RESET_GIMBAL
    command_process_mode: _CmdProcessMode = _CmdProcessMode.WAIT_FOR_READY


@dataclass
class _SetAttitude:
    attitude: Quaternion

    command_type: _CmdType = _CmdType.SET_ATTITUDE
    command_process_mode: _CmdProcessMode = _CmdProcessMode.WAIT_FOR_READY


_Command = Union[_MountConfigure, _ResetGimbal, _SetAttitude]


# Gremsy Mavlink implementation docs: https://github.com/Gremsy/gSDK/tree/master/doc
class GimbalGremsy(Gimbal):
    """A Mavlink heartbeat must be sent to the gimbal before it will enter mavlink mode and begin
    sending and responding to messages from this component
    """
    def __init__(
            self,
            controller: MavlinkNode,
            gimbal_axes: GimbalAxes
        ):
        self._controller: MavlinkNode = controller
        self._gimbal_axes: GimbalAxes = gimbal_axes
        # Gremsy specifies this will be the ID of their gimbal
        self._gimbal_device_id: ComponentId = ComponentId(mavlink2.MAV_COMP_ID_GIMBAL)

        # assumes the gimbal adopts the system ID of the first system to send a heartbeat
        # the Gremsy gimbal software doesn't fully enter Mavlink mode and begin sending heartbeats
        # until it receives a heartbeat itself
        self._gimbal_system_id: SystemId = controller.system_id

        self._mavlink_protocol_handler = mavlink2.MAVLink(
            "", 
            self._controller.system_id,
            self._controller.component_id
        )

        self._mavlink_protocol_handler_mega = mavlink2_mega.MAVLink(
            "", 
            self._controller.system_id,
            self._controller.component_id
        )

        self._gimbal_status_lock: RLock = RLock()
        self._gimbal_status: GimbalStatus = GimbalStatus.NOT_CONNECTED

        self._do_mount_configure_status_lock: RLock = RLock()
        self._do_mount_configure_status_complete: bool = False

        # call after locks are initialized
        super().__init__()

        self._command_to_packer_map: Mapping[_CmdType, Callable] = {
            _CmdType.MOUNT_CONFIGURE: self._pack_do_mount_configure,
            _CmdType.RESET_GIMBAL: self._pack_set_neutral_attitude,
            _CmdType.SET_ATTITUDE: self._pack_do_mount_control
        }

        self._received_message_to_callback: Mapping[int, Callable] = {
            mavlink2.MAVLINK_MSG_ID_COMMAND_ACK:
            self._receive_command_ack,
            mavlink2.MAVLINK_MSG_ID_HEARTBEAT:
            self._receive_heartbeat,
            mavlink2.MAVLINK_MSG_ID_MOUNT_ORIENTATION:
            self._receive_mount_orientation,
            mavlink2_mega.MAVLINK_MSG_ID_MOUNT_STATUS:
            self._receive_mount_status,
            mavlink2.MAVLINK_MSG_ID_RAW_IMU:
            self._receive_raw_imu,
            mavlink2.MAVLINK_MSG_ID_SYS_STATUS:
            self._receive_sys_status
        }

        self._connection: MavlinkRouterTCP = MavlinkRouterTCP(
            gained_connection_callback=self._gained_connection,
            lost_connection_callback=self._lost_connection,
            receive_message_callback=self.receive_message_callback
        )
        self._connection.start()

        # we need mavlink router to process a command to start sending heartbeats back to the source
        # system and component id
        self._set_default_mount_configuration()


    def _calculate_attitude_difference(
            self,
            current_attitude: Quaternion,
            desired_attitude: Quaternion) -> Quaternion:
        """This operation first applies the rotation from the coordinate frame of the
        vector after rotation by curren_attitude is applied (v') back to the original reference 
        frame (v). Rotation from v' to v requires the inverse (or conjugate for unit quaternions).
        It then applies the second rotation of the desired_attitude from v into v'' which when 
        composed yields the total rotation from v' into v''.
        """
        return Quaternion(quaternion_multiply(
            quaternion0=desired_attitude,
            quaternion1=(
                -current_attitude[0],
                -current_attitude[1],
                -current_attitude[2],
                current_attitude[3]
            )
        ))


    def _gained_connection(self):
        with self._gimbal_status_lock:
            self._gimbal_status = GimbalStatus.CONNECTED_BUT_NOT_IN_CONTROL

        rospy.loginfo("GimbalGremsy - gained connection to gimbal")


    def _lost_connection(self):
        with self._gimbal_status_lock:
            self._gimbal_status = GimbalStatus.NOT_CONNECTED

        rospy.logwarn("GimbalGremsy - lost connection to gimbal")


    def _manage_command_process_mode(self, message: _Command, command_queue: Queue) -> bool:
        """Returns True if the command queue should exit before processing a command
        """
        if message.command_process_mode == _CmdProcessMode.ALWAYS:
            return False

        with self._gimbal_status_lock:
            if self._gimbal_status == GimbalStatus.READY:
                return False

        if message.command_process_mode == _CmdProcessMode.SKIP_IF_NOT_READY:
            return True

        if message.command_process_mode == _CmdProcessMode.WAIT_FOR_READY:
            command_queue.put(message)
            return True

        # default to processing the command
        return False


    def _on_exit(self):
        self._connection.close()


    def _process_command_queue(self):
        try:
            msg_to_send: _Command = self._command_queue.get(timeout=1)
        except Empty:
            rospy.logdebug("GimbalGremsy - _command_queue.get Empty exception")
            return

        if self._manage_command_process_mode(msg_to_send, self._command_queue):
            return

        self._connection.send(self._command_to_packer_map[msg_to_send.command_type](msg_to_send))


    def _process_messages(self):
        message_id: int
        message: mavlink2.MAVLink_message
        message_id, message = self._received_messages_queue.get()
        self._received_message_to_callback[message_id](message)


    def _pack_do_mount_configure(self, msg: _MountConfigure) -> bytes:
        return(mavlink2.MAVLink_command_long_message(
                target_system=self._gimbal_system_id,
                target_component=self._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONFIGURE,
                confirmation=0,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                param5=msg.roll_mode.value,
                param6=msg.pitch_mode.value,
                param7=msg.yaw_mode.value
            ).pack(self._mavlink_protocol_handler)
        )


    def _quaternion_to_axis_angle(self, attitude: Quaternion) -> Tuple[float, float, float]:
        """
        Returns
        -------
            (x_roll, y_pitch, z_yaw) in deg
        """
        r = attitude[3]
        v = (attitude[0], attitude[1], attitude[2])
        # sqrt of the dot product of v with itself
        norm_v = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

        # rotation angle
        theta = 2 * math.atan2(norm_v, r)

        if theta == 0:
            return (0, 0, 0)

        w = tuple(dim / math.sin(theta / 2) for dim in v)

        return tuple(dim * theta * (180 / math.pi) for dim in w)


    def _pack_do_mount_control(self, msg: _SetAttitude) -> bytes:
        axis_angle_deg = self._quaternion_to_axis_angle(msg.attitude)

        return(mavlink2.MAVLink_command_long_message(
                target_system=self._gimbal_system_id,
                target_component=self._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONTROL,
                confirmation=0,
                param1=axis_angle_deg[1], # pitch y-axis out right of drone
                param2=axis_angle_deg[0], # roll x-axis out front of drone
                param3=axis_angle_deg[2], # yaw z-axis down
                param4=0,
                param5=0,
                param6=0,
                param7=0
            ).pack(self._mavlink_protocol_handler)
        )


    def _pack_set_neutral_attitude(self, msg: _ResetGimbal) -> bytes:
        with self._current_attitude_lock:
            current_attitude = self._current_attitude

        axis_angle_deg = self._quaternion_to_axis_angle(
            self._calculate_attitude_difference(
                current_attitude,
                Quaternion((0, 0, 0, 1))
            )
        )

        return(mavlink2.MAVLink_command_long_message(
                target_system=self._gimbal_system_id,
                target_component=self._gimbal_device_id,
                command=mavlink2.MAV_CMD_DO_MOUNT_CONTROL,
                confirmation=0,
                param1=axis_angle_deg[1], # pitch y-axis out right of drone
                param2=axis_angle_deg[0], # roll x-axis out front of drone
                param3=axis_angle_deg[2], # yaw z-axis down
                param4=0,
                param5=0,
                param6=0,
                param7=0
            ).pack(self._mavlink_protocol_handler)
        )


    def _receive_command_ack(
        self,
        msg: mavlink2.MAVLink_command_ack_message
    ):
        if (msg.command == mavlink2.MAV_CMD_DO_MOUNT_CONFIGURE
            and
            msg.result == mavlink2.MAV_RESULT_ACCEPTED):
            with self._do_mount_configure_status_lock:
                self._do_mount_configure_status_complete = True
            rospy.loginfo("GimbalGremsy - do mount configure confirmed complete")


    def _receive_heartbeat(
        self,
        msg: mavlink2.MAVLink_heartbeat_message
    ):
        if msg.get_srcComponent() == mavlink2.MAV_COMP_ID_GIMBAL:
            with self._gimbal_status_lock:
                self._gimbal_status = GimbalStatus.READY


    def _receive_mount_orientation(
        self,
        msg: mavlink2.MAVLink_mount_orientation_message
    ):
        """Currently do nothing with the mount orientation data when received
        """
        # assumes the gimbal coordinate frame orientation matches the airframe's forward (roll),
        # right (pitch), down (yaw) right-hand frame
        # assumes that 0 angles align with no rotations in the coordinate frame and are half way
        # between the max rotation in either direction

        # handle situation where axis is invalid for a particular gimbal
        if math.isnan(msg.pitch):
            msg.pitch = 0
        pitch_rad = msg.pitch * (math.pi / 180)

        if math.isnan(msg.roll):
            msg.roll = 0
        roll_rad = msg.roll * (math.pi / 180)

        if math.isnan(msg.yaw):
            msg.yaw = 0
        yaw_rad = msg.yaw * (math.pi / 180)

        # use rotating axes for conversion since gimbal encoder axes will be rotated by previous
        # gimbal rotations of other axes
        current_attitude = Quaternion(tuple(quaternion_from_euler(
                ai=roll_rad,
                aj=pitch_rad,
                ak=yaw_rad,
                axes='rxyz'
            )))

        with self._current_attitude_lock:
            self._current_attitude = current_attitude


    # https://github.com/mavlink/c_library_v1/blob/master/ardupilotmega/mavlink_msg_mount_status.h
    def _receive_mount_status(
        self,
        msg: mavlink2_mega.MAVLink_mount_status_message
    ):
        pass
        """NOTE: using the mount_orientation message to set current attitude instead of mount_status
        # assumes the gimbal coordinate frame orientation matches the airframe's forward (roll),
        # right (pitch), down (yaw) right-hand frame
        # assumes that 0 angles align with no rotations in the coordinate frame and are half way
        # between the max rotation in either direction
        pitch_rad = msg.pointing_a / 100 * (math.pi / 180)
        roll_rad = msg.pointing_b / 100 * (math.pi / 180)
        yaw_rad = msg.pointing_c / 100 * (math.pi / 180)

        # use rotating axes for conversion since gimbal encoder axes will be rotated by previous
        # gimbal rotations of other axes
        current_attitude = Quaternion(tuple(quaternion_from_euler(
                ai=roll_rad,
                aj=pitch_rad,
                ak=yaw_rad,
                axes='rxyz'
            )))

        with self._current_attitude_lock:
            self._current_attitude = current_attitude
        """


    def _receive_raw_imu(
        self,
        msg: mavlink2.MAVLink_raw_imu_message
    ):
        """Currently do nothing with the raw imu data when received
        """
        pass


    def _receive_sys_status(
        self,
        msg: mavlink2.MAVLink_sys_status_message
    ):
        """Currently do nothing with the sys status data when received
        """
        pass


    def _set_default_mount_configuration(self):
        self._command_queue.put(_MountConfigure())


    def get_gimbal_status(self) -> GimbalStatus:
        with self._gimbal_status_lock:
            return self._gimbal_status


    def reaquire_control(self):
        """Left unimplemented as reconnects are automatically handled in the queue processing
        threads
        """
        return


    def receive_message_callback(self, message: bytes):
        decoded_message: Optional[mavlink2.MAVLink_message] = None

        try:
            decoded_message: mavlink2.MAVLink_message = self._mavlink_protocol_handler.decode(
                array.array('B', message)
            )
        except mavlink2.MAVError:
            try:
                rospy.logdebug("GimbalGremsy - mavlink2_mega needed for message decoding")
                decoded_message: mavlink2_mega.MAVLink_message = (
                    self._mavlink_protocol_handler_mega.decode(
                    array.array('B', message)
                ))
            except mavlink2_mega.MAVError as mav_error:
                rospy.logdebug(f"GimbalGremsy - {mav_error}")

        if decoded_message is not None:
            self._received_messages_queue.put(
                (
                    decoded_message.get_msgId(),
                    decoded_message
                )
            )


    def set_attitude(self, attitude: Quaternion):
        with self._do_mount_configure_status_lock:
            if not self._do_mount_configure_status_complete:
                self._command_queue.put(_MountConfigure())

        current_attitude = self.get_attitude()

        if current_attitude is None:
            # attitude is neutral as assumed no rotations yet applied
            current_attitude = Quaternion((0, 0, 0, 1))

        self._command_queue.put(
            _SetAttitude(self._calculate_attitude_difference(
                current_attitude,
                attitude
            )
        ))


    def set_neutral_attitude(self):
        self._command_queue.put(_ResetGimbal())
