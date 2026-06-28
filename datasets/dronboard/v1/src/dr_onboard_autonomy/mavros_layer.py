#!/usr/bin/env python3
import enum

from copy import copy
from enum import Enum
from threading import Event, Lock, Thread
from typing import List, Optional, Tuple, Union

import rospy

from geographic_msgs.msg import GeoPoseStamped
from mavros import mavlink
from mavros_msgs.msg import GlobalPositionTarget, Mavlink, ParamValue, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

from .drone_data import Data
from .mavlink import MavlinkSender, HeartbeatSender, MavType, MavState
from .gimbal import Gimbal
from .frames import AttitudeData, PositionData


from pymavlink.dialects.v20 import common

"""

>>from mavros_layer import MAVROSDrone
>>my_drone = MAVROSDrone("birdy0')
>>my_drone.arm()
"""

class SetpointType(Enum):
    LLA = enum.auto()
    NED_POSITION = enum.auto()
    NED_VELOCITY = enum.auto()


Vec3 = Tuple[float, float, float]
Yaw = float
SetpointData = Tuple[Optional[SetpointType], Optional[Vec3], Optional[Yaw]]


class MAVROSDrone:
    def __init__(self, uav_name, simulate_gcs_heartbeat=True, init_gimbal=True):
        """Create a Drone. You can control the drone with this object.

        Call this object's methods,when you want to control the drone. Most of these methods cause
        a MAVLink message to get sent to the drone.

        When simulate_gcs_heartbeat is True, this object will send a heartbeat so that the flight
        controller thinks QGroundControl is connected. You can set this to false when QGroundControl
        is connected to the Flight Control during the flight operation.

        When init_gimbal is True, this object creates a gimbal object. You can use the gimbal object
        to control the drone's gimbal.

        """
        self.uav_name = uav_name

        self.system_id = rospy.get_param('mavros/target_system_id')
        self.fc_component_id = rospy.get_param('mavros/target_component_id')
        self.component_id = mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
        
        self._setpoint_type: Optional[SetpointType] = None
        self._setpoint_vec: Optional[Vec3] = None
        self._setpoint_yaw: Optional[Yaw] = None
        
        self.serv_timeout = 10

        self.position_data = PositionData()
        self.attitude_data = AttitudeData()

        self._data = Data()
        self._data.uavid = self.uav_name
        self._lock = Lock()

        self._mavlink_pub = rospy.Publisher("mavlink/to", Mavlink, queue_size=1)
        self._arm_sp = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._mode_sp = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self._param_set_sp = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self._param_get_sp = rospy.ServiceProxy("mavros/param/get", ParamGet)

        # For setpoint publishers we want a queue_size of 1.
        # Because new setpoint messages should override old ones, if they've not been sent.
        # otherwise the drone could be slower to respond to set points
        # more info http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        self._setpoint_local_pub = rospy.Publisher(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self._setpoint_global_pub = rospy.Publisher(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=1
        )

        self._heartbeat_senders: List[HeartbeatSender] = []
        if simulate_gcs_heartbeat:
            # since we're simulating a GCS, we need a System ID for it 
            # A system_id near 255 is recommend by the mavlink protocol:
            # https://mavlink.io/en/guide/routing.html    
            # since QGroundControl defaults to 255 we will set ours to 254
            gcs_system_id=254
            hb_mavlink_sender = MavlinkSender(gcs_system_id, mavlink2.MAV_COMP_ID_MISSIONPLANNER, self._mavlink_pub)
            # the send frequency of 2 is based on this section of mavlink website says:
            # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
            gcs_hb = HeartbeatSender(hb_mavlink_sender, component_type=MavType.GCS, send_frequency=2)
            gcs_hb.mav_state = MavState.ACTIVE
            self._heartbeat_senders.append(gcs_hb)
        
        self.mavlink_sender = MavlinkSender(system_id=self.system_id, component_id=self.component_id, mavlink_pub=self._mavlink_pub)
        self.onboard_heartbeat = HeartbeatSender(self.mavlink_sender, MavType.ONBOARD_CONTROLLER)
        self._heartbeat_senders.append(self.onboard_heartbeat)
        self.gimbal = False
        if init_gimbal:
            self.gimbal = Gimbal(self.mavlink_sender)
    
    def start(self):
        for hb in self._heartbeat_senders:
            hb.start()
        
        self.onboard_heartbeat.mav_state = MavState.ACTIVE
        if self.gimbal:
            self.gimbal.start()

    def stop(self):
        pass

    def update_data(self, key, val):
        # TODO check if we access this method from multiple threads
        # TODO check if we can make the change in-place?
        with self._lock:
            data = copy(self._data)
            setattr(data, key, val)
            self._data = data

    def get_data(self, key):
        """returns value for specified drone data attribute
        """
        with self._lock:
            return getattr(self._data, key, None)
    
    @property
    def setpoint(self) -> SetpointData:
        return (self._setpoint_type, self._setpoint_vec, self._setpoint_yaw)
    
    @property
    def data(self):
        with self._lock:
            return copy(self._data)

    def _set_param(self, param_name, real_value=0.0, integer_value=0):
        msg = ParamValue()
        msg.integer = int(integer_value)
        msg.real = float(real_value)

        rospy.wait_for_service("mavros/param/set", self.serv_timeout)
        try:
            response = self._param_set_sp(param_id=param_name, value=msg)
        except rospy.ServiceException as exc:
            return False
        else:
            return response.success

    def _get_param(self, param_name) -> Union[ParamGet, bool]:
        rospy.wait_for_service("mavros/param/get", self.serv_timeout)

        try:
            response = self._param_get_sp(param_name)
        except rospy.ServiceException as exc:
            rospy.logerr("Unable to get parameter {}".format(param_name))
            return False
        else:
            return response
    
    def _get_param_value(self, param_name: str) -> ParamValue:
        """calls the mavros/param/get service and returns the ParamValue

        Throws an exception if not successful.
        """
        response: ParamGet = self._param_get_sp(param_name)
        if not response.success:
            raise Exception(f"could not read PX4 parameter {param_name}")

        return response.value
    
    def get_param_integer(self, param_name: str) -> int:
        """Get an integer-valued parameter from the flight controller.
        Returns the parameter's value or raises rospy.ServiceException.

        Params:
            param_name: the name of the parameter

        Note: You can find the parameter reference page at:
        https://docs.px4.io/main/en/advanced_config/parameter_reference.html
        """
        value: ParamValue = self._get_param_value(param_name)
        return int(value.integer)

    def get_param_real(self, param_name: str) -> float:
        """Get an real-valued parameter from the flight controller.
        Returns the parameter's value or raises an Exception.

        Params:
            param_name: the name of the parameter

        Note: You can find the parameter reference page at:
        https://docs.px4.io/main/en/advanced_config/parameter_reference.html
        """
        value: ParamValue = self._get_param_value(param_name)
        return float(value.real)

    def _set_mode(self, mode):
        rospy.wait_for_service("mavros/set_mode", self.serv_timeout)
        try:
            response = self._mode_sp(0, mode)
        except rospy.ServiceException as exc:
            return False
        else:
            return response.mode_sent

    def arm(self) -> bool:
        rospy.wait_for_service("mavros/cmd/arming", self.serv_timeout)
        try:
            response = self._arm_sp(True)
        except rospy.ServiceException as exc:
            rospy.logfatal("Could not arm")
            rospy.logfatal(str(exc))
            return False
        else:
            return response.success

    def disarm(self):
        rospy.wait_for_service("mavros/cmd/arming", self.serv_timeout)
        try:
            response = self._arm_sp(False)
        except rospy.ServiceException as exc:
            return False
        else:
            return response.success

    def takeoff(self):
        return self._set_mode("AUTO.TAKEOFF")

    def hover(self):
        return self._set_mode("AUTO.LOITER")

    def land(self):
        resp = self._set_mode("AUTO.LAND")
        return resp

    def set_mode(self, mode: str):
        """Change the flight controller's mode

        The mode string must be one of these values:
        - POSCTL
        - STABILIZED
        - OFFBOARD
        - AUTO.MISSION
        - AUTO.RTL
        - AUTO.LAND
        - AUTO.LOITER

        Return True if successful or False if not.
        """
        # These docs list other possible mode values:
        # http://wiki.ros.org/mavros/CustomModes

        is_mode_valid = mode in {
            "POSCTL",
            "STABILIZED",
            "OFFBOARD",
            "AUTO.MISSION",
            "AUTO.RTL",
            "AUTO.LAND",
            "AUTO.LOITER",
        }
        if not is_mode_valid:
            # Maybe this should:
            # raise ValueError(f"Invalid mode: '{mode}'")
            return False

        if mode == "AUTO.LOITER":
            return self.hover()
        elif mode == "AUTO.LAND":
            return self.land()
        elif mode == "OFFBOARD":
            return self._set_mode(mode)
        elif mode == "AUTO.MISSION":
            return self._set_mode(mode)
        elif mode == "AUTO.RTL":
            return self._set_mode(mode)
        else:
            # mode could be POSCTL or STABILIZED
            # these modes take input from the RC transmitter
            # should we set an action in case RC signal is lost?
            # NAV_RCL_ACT lets us specify the action we want to take
            # self._set_param("NAV_RCL_ACT", integer_value=2)
            return self._set_mode(mode)

    def send_setpoint(
        self, lla=None, ned_position=None, ned_velocity=None, yaw=0.0, is_yaw_set=False
    ):
        """Send a setpoint command to the flight controller
        Important: lla, ned_position, ned_velocity are mutually exclusive.
        do NOT call this method giving more than one of those values. Otherwise
        the behavior of this function is undefined.

        Arguments
        =========

        lla: Tuple[float, float, float]
            Sends a global position set point with latitue, longitude and altitude values

        ned_position: Tuple[float, float, float]
            Sends a local position set point with north, east and down values. (units in meters)

        ned_velocity: Tuple[float, float, float]
            Sends a local velocity set point with north, east, and down values. (units in meters per second)

        yaw: float
            Specifies the aircraft's yaw direction in ENU. The drone will face this direction while flying (units in radians)

        is_yaw_set: bool
            if True, then the yaw value will be recognized. Otherwise the yaw value is ignored

        """
        exclusive_args = [lla, ned_position, ned_velocity]
        exclusive_args = filter(lambda arg: arg is not None, exclusive_args)
        exclusive_args = list(exclusive_args)
        if len(exclusive_args) != 1:
            err_msg = f"send_setpoint called with wrong number of mutually exclusive arguments. You must provide exactly one of these: lla={lla}, ned_position={ned_position}, ned_velocity={ned_velocity}"
            rospy.logerr(err_msg)
            # TODO decide if it's better to raise a value error:
            # raise ValueError(err_msg)
            return False

        def is_arg_ok(tup):
            error_message = f"error in MAVROSDrone.send_setpoint(): you must provide a tuple holding three numbers (you called me with: {str(type(tup))} = '{tup}')"
            if type(tup) != tuple:
                rospy.logerr("type is wrong")
                rospy.logerr(error_message)

                # raise ValueError(error_message)
                return False
            if len(tup) != 3:
                rospy.logerr("tuple length is wrong")
                rospy.logerr(error_message)
                # raise ValueError(error_message)
                return False

            is_all_numbers = [
                type(tup[0]) in [int, float],
                type(tup[1]) in [int, float],
                type(tup[2]) in [int, float],
            ]

            if not all(is_all_numbers):
                rospy.logerr("tuple elements are wrong")
                rospy.logerr(error_message)
                # raise ValueError(error_message)
                return False

            return True

        if is_yaw_set:
            self._setpoint_yaw = yaw
        else:
            self._setpoint_yaw = None

        set_point = None
        pub = None
        if lla is not None and is_arg_ok(lla):
            set_point = self._build_global_setpoint2(*lla, yaw=yaw)
            self._setpoint_type = SetpointType.LLA
            self._setpoint_vec = lla
            pub = self._setpoint_global_pub
        else:
            pub = self._setpoint_local_pub
            if ned_position is not None and is_arg_ok(ned_position):
                set_point = self._build_local_setpoint(*ned_position, is_velocity=False)
                self._setpoint_type = SetpointType.NED_POSITION
                self._setpoint_vec = ned_position
            elif ned_velocity is not None and is_arg_ok(ned_velocity):
                set_point = self._build_local_setpoint(*ned_velocity)
                self._setpoint_type = SetpointType.NED_VELOCITY
                self._setpoint_vec = ned_velocity
            if is_yaw_set:
                set_point.yaw = yaw
                set_point.type_mask = (
                    set_point.type_mask ^ 1024
                )  # zero out the IGNORE_YAW bit

        if set_point is None or pub is None:
            return False

        pub.publish(set_point)
        return True

    @staticmethod
    def _build_local_setpoint(north, east, down, is_velocity=True):
        """
        Build a PositionTarget object.
        if is_velocity is True, then the message controls the drone's velcity
        otherwise it controls the drone's position
        """
        # This message is documented here:
        # http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html
        pos_target = PositionTarget()
        pos_target.header.stamp = rospy.Time.now()
        pos_target.coordinate_frame = 1  # FRAME_LOCAL_NED = 1

        # To set the type_mask we need to know what the value means
        # It's a bit mask. Each bit controls whether or not a field is ignored.
        # if the bit is 1 then the field corresponding to that bit is ignored
        #
        # First we will set all the bits to 1.
        # Later we will zero out the bits we need
        pos_target.type_mask = 0b111_111_111_111

        if is_velocity:
            # we need to zero out the velocity bits
            pos_target.type_mask = pos_target.type_mask ^ 0b111_000
            pos_target.velocity.x = north
            pos_target.velocity.y = east
            pos_target.velocity.z = down
        else:
            # we need to zero out the position bits
            pos_target.type_mask = pos_target.type_mask ^ 0b111
            pos_target.position.x = north
            pos_target.position.y = east
            pos_target.position.z = down
        return pos_target

    @staticmethod
    def _build_global_setpoint(latitude, longitude, altitude):
        """Build a message for /mavros/setpoint_raw/global"""
        global_target = GlobalPositionTarget()
        global_target.header.stamp = rospy.Time.now()
        # g.coordinate_frame = 0 where MAV_FRAME_GLOBAL = 0
        # MAV_FRAME_GLOBAL is the only one PX4 supports
        # Here is what the docs say about MAV_FRAME_GLOBAL:
        #   Global (WGS84) coordinate frame + MSL altitude.
        #   First value / x: latitude,
        #   second value / y: longitude,
        #   third value / z: positive altitude over mean sea level (MSL).
        global_target.coordinate_frame = 0
        # global_target.type_mask = 0b111_111_111_000

        IGNORE_LATITUDE = 1  # Position ignore flags
        IGNORE_LONGITUDE = 2
        IGNORE_ALTITUDE = 4
        IGNORE_VX = 8  # Velocity vector ignore flags
        IGNORE_VY = 16
        IGNORE_VY = 32
        IGNORE_AFX = 64  # Acceleration/Force vector ignore flags
        IGNORE_AFY = 128
        IGNORE_AFZ = 256
        FORCE = 512  # Force in af vector flag
        IGNORE_YAW = 1024
        IGNORE_YAW_RATE = 2048

        mask = 0
        for field in [
            IGNORE_VX,
            IGNORE_VY,
            IGNORE_VY,
            IGNORE_AFX,
            IGNORE_AFY,
            IGNORE_AFZ,
            FORCE,
            IGNORE_YAW,
            IGNORE_YAW_RATE,
        ]:
            mask = mask | field

        global_target.type_mask = mask
        global_target.latitude = latitude
        global_target.longitude = longitude
        # This altitude is interpreted as above mean seal level (AMSL).
        # But our GPS is telling us altitude above the WGS-84 ellipsoid.
        global_target.altitude = altitude
        return global_target

    @staticmethod
    def _build_global_setpoint2(latitude, longitude, altitude, yaw=0.0):
        """Builds a message for the /mavros/setpoint_position/global

        MAVROS interprets altitude as above mean seal level (AMSL).
        Our GPS sensor is telling us altitude above the WGS-84 ellipsoid.
        """
        geo_pose_setpoint = GeoPoseStamped()
        geo_pose_setpoint.header.stamp = rospy.Time.now()
        geo_pose_setpoint.pose.position.latitude = latitude
        geo_pose_setpoint.pose.position.longitude = longitude
        geo_pose_setpoint.pose.position.altitude = altitude

        roll = 0.0
        pitch = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        geo_pose_setpoint.pose.orientation.x = q[0]
        geo_pose_setpoint.pose.orientation.y = q[1]
        geo_pose_setpoint.pose.orientation.z = q[2]
        geo_pose_setpoint.pose.orientation.w = q[3]

        return geo_pose_setpoint

    def _build_global_setpoint3(latitude, longitude, altitude, yaw=0.0):
        """Builds a setpoint message for the "/mavlink/to" topic.

        MAVROS interprets altitude as above mean seal level (AMSL).
        Our GPS sensor is telling us altitude above the WGS-84 ellipsoid.
        """

        mavutil.mavlink.MAVLink
        
    def gimbal_rotation(self, roll, pitch, yaw):
        
        #msg = common.MAVLink_command_long_message(target_system=0, target_component=0, command=205, confirmation=1, param1=-90, param2=0, param3=0, param4=0, param5=0, param6=0, param7=2)
        #msg.pack(self._mav)
        #self._mavlink_pub.publish(mavlink.convert_to_rosmsg(msg))

        connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        connection.wait_heartbeat()
        connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 1, pitch, roll, yaw, 0, 0, 0, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        connection.close()


