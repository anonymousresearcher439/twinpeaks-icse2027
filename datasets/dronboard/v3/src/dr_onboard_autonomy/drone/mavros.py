import math
import time
from typing import Dict, Optional, Tuple, TypedDict, Union

from std_msgs.msg import Header
from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import Mavlink, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool,
    CommandBoolRequest,
    CommandBoolResponse,
    ParamGet,
    ParamGetResponse,
    ParamSet,
    ParamSetResponse,
    SetMode,
    SetModeRequest,
    SetModeResponse
)
from pymavlink.dialects.v20 import common as mavlink2
import rospy

from dr_onboard_autonomy.models import (
    CopterDrone,
    CopterParameters,
    EnuYaw,
    FCUCopterMode,
    Gimbal,
    LlaPosition,
    NedPosition,
    NedVelocity
)
from dr_onboard_autonomy.mavlink import (
    HeartbeatSender,
    MavlinkNode,
    MavlinkSender,
    MavState,
    MavType
)
# since we're simulating a GCS, we need a System ID for it 
# A system_id near 255 is recommend by the mavlink protocol:
# https://mavlink.io/en/guide/routing.html    
# since QGroundControl defaults to 255 we will set ours to 254
_GCS_SYSTEM_ID = 254
_SERVICE_PROXY_TIMEOUT = 10


class ArdupilotCopterMavrosDrone(CopterDrone):
    # http://wiki.ros.org/mavros/CustomModes
    FCU_MODE_MAP = (
        (FCUCopterMode.ALT_HOLD, "ALT_HOLD"),
        (FCUCopterMode.LAND, "LAND"),
        (FCUCopterMode.LOITER, "LOITER"),
        (FCUCopterMode.MISSION, "AUTO"),
        (FCUCopterMode.OFFBOARD, "GUIDED"),
        (FCUCopterMode.POS_HOLD, "POSHOLD"),
        (FCUCopterMode.RTL, "RTL"),
        (FCUCopterMode.STABILIZE, "STABILIZE")
    )

    class FCUMavlinkMessages(TypedDict):
        msg_id: int
        frequency: int

    FCU_MESSAGES_TO_REQUEST: Tuple[FCUMavlinkMessages] = (
        # mavros/global_position
        {"msg_id": mavlink2.MAVLINK_MSG_ID_GPS_RAW_INT, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "frequency": 25},
        {
            "msg_id": mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET,
            "frequency": 25
        },
        # mavros/state
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SYS_STATUS, "frequency": 1},
        # mavros/extended_state
        {"msg_id": mavlink2.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, "frequency": 1},
        # mavros/battery
        {"msg_id": mavlink2.MAVLINK_MSG_ID_BATTERY_STATUS, "frequency": 1},
        # mavros/estimator_status
        {"msg_id": mavlink2.MAVLINK_MSG_ID_ESTIMATOR_STATUS, "frequency": 1},
        # mavros/imu
        {"msg_id": mavlink2.MAVLINK_MSG_ID_ATTITUDE, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_HIGHRES_IMU, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_RAW_IMU, "frequency": 25},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SCALED_IMU, "frequency": 10},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SCALED_PRESSURE, "frequency": 10},
        # mavros/rc
        {"msg_id": mavlink2.MAVLINK_MSG_ID_RC_CHANNELS_RAW, "frequency": 1},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_RC_CHANNELS, "frequency": 1},
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, "frequency": 1},
        # mavros/time_reference
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SYSTEM_TIME, "frequency": 1},
    )


    def __init__(self, gimbal: Gimbal, mavlink_node: MavlinkNode, uav_name: str):
        super().__init__(gimbal=gimbal, uav_name=uav_name)

        self.mavlink_node = mavlink_node

        self._arm_sp: Optional[CommandBool] = None
        self._param_get_sp: Optional[ParamGet] = None
        self._param_set_sp: Optional[ParamSet] = None
        self._set_mode_sp: Optional[SetMode] = None

        # For setpoint publishers we want a queue_size of 1 because new setpoint messages should
        # override old ones, if they've not been sent.
        # Otherwise the drone could be slower to respond to set points
        # more info http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        self._setpoint_local_pub = rospy.Publisher(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self._setpoint_global_pub = rospy.Publisher(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=1
        )
        self._mavlink_pub = rospy.Publisher(
            "mavlink/to", Mavlink, queue_size=1
        )

        # simulated ground control station heartbeat in case we don't have a GCS connected
        self._gcs_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                _GCS_SYSTEM_ID,
                mavlink2.MAV_COMP_ID_MISSIONPLANNER,
                self._mavlink_pub
            ),
            component_type=MavType.GCS,
            send_frequency=2 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._gcs_hb.mav_state = MavState.ACTIVE
        self._gcs_hb.start()

        # mavlink heartbeat from onboard controller to fcu
        self._controller_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                mavlink_node.system_id,
                mavlink_node.component_id,
                self._mavlink_pub
            ),
            component_type=MavType.ONBOARD_CONTROLLER,
            send_frequency=1 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._controller_hb.mav_state = MavState.ACTIVE
        self._controller_hb.start()

        self._mavlink_to_fcu = MavlinkSender(
            mavlink_node.system_id,
            mavlink_node.component_id,
            self._mavlink_pub
        )


    def arm(self) -> bool:
        try:
            response: CommandBoolResponse = self._arm_sp(CommandBoolRequest(value=True))
        except rospy.ServiceException as service_exception:
            rospy.logfatal(
                f"ArdupilotCopterMavrosDrone - arm exception: {service_exception}"
            )
            return False

        return response.success


    def get_fcu_parameter(self, name: str) -> Optional[Union[float, int]]:
        try:
            response: ParamGetResponse = self._param_get_sp(param_id=name)
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"ArdupilotCopterMavrosDrone - get fcu parameter exception: {service_exception}"
            )
            return

        if not response.success:
            return

        if response.value.real != 0.0:
            return response.value.real

        return response.value.integer


    def set_fcu_parameter(self, name: str, value: Union[float, int]) -> bool:
        if isinstance(value, float):
            msg = ParamValue(
                integer=0,
                real=value
            )
        else:
            msg = ParamValue(
                integer=value,
                real=0.0
            )

        try:
            response: ParamSetResponse = self._param_set_sp(
                param_id=name,
                value=msg
            )
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"ArdupilotCopterMavrosDrone - set fcu parameter exception: {service_exception}"
            )
            return False

        return response.success


    def start(self):
        rospy.wait_for_service("mavros/cmd/arming", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/get", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/set", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/set_mode", _SERVICE_PROXY_TIMEOUT)
    
        self._arm_sp: CommandBool = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._param_get_sp: ParamGet = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self._param_set_sp: ParamSet = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self._set_mode_sp: SetMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        for _ in range(300):
            # For some reason the RC_OPTIONS parameter is not available immediately after starting mavros
            # so we will keep trying to get it until it is available
            rc_options = self.get_fcu_parameter("RC_OPTIONS")
            if rc_options is not None:
                break
            rospy.logwarn("RC_OPTIONS parameter not available yet, waiting...")
            time.sleep(1)
        if rc_options is None:
            # if we were never able to get the RC_OPTIONS parameter, we should shutdown the program
            rospy.logerr("RC_OPTIONS parameter not available, unable to start dr_onboard")
            rospy.signal_shutdown()
            exit(111)
        
        self.set_fcu_parameter("RC_OPTIONS", rc_options & 0b1111111111011111) # clear bit 5
        self._request_mavlink_messages()




    def send_mavlink(self, mavlink_message: mavlink2.MAVLink):
        """Given a mavlink message, send it to the FCU

        Note this will update the message with the correct target system and component.
        """
        mavlink_message.target_system = self.mavlink_node.system_id
        mavlink_message.target_component = mavlink2.MAV_COMP_ID_AUTOPILOT1
        rospy.loginfo(f"Sending mavlink message: {mavlink_message}")
        self._mavlink_to_fcu.send(mavlink_message)


    def _request_mavlink_messages(self):
        for mavlink_msg in self.FCU_MESSAGES_TO_REQUEST:
            self._mavlink_to_fcu.send(
                mavlink2.MAVLink_command_long_message(
                    target_system=self.mavlink_node.system_id,
                    target_component=mavlink2.MAV_COMP_ID_AUTOPILOT1,
                    command=mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
                    confirmation=0,
                    param1=mavlink_msg["msg_id"],
                    param2=(1 / mavlink_msg["frequency"]) * 1E6,
                    param3=0,
                    param4=0,
                    param5=0,
                    param6=0,
                    param7=1 # address of requestor
                )
            )
            # we must add a time buffer between requests or the FCU will fail to process them
            time.sleep(0.2)


    def get_available_fcu_modes(self) -> Tuple[FCUCopterMode]:
        return (
            FCUCopterMode.ALT_HOLD,
            FCUCopterMode.LAND,
            FCUCopterMode.LOITER,
            FCUCopterMode.MISSION,
            FCUCopterMode.OFFBOARD,
            FCUCopterMode.POS_HOLD,
            FCUCopterMode.RTL,
            FCUCopterMode.STABILIZE
        )


    def _send_lla_setpoint(
        self,
        lla: LlaPosition,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        # MAVROS interprets altitude as above mean seal level (AMSL).
        lla = lla.to_amsl()

        pose = GeoPose()
        pose.position.latitude = lla.latitude
        pose.position.longitude = lla.longitude
        pose.position.altitude = lla.altitude

        pose.orientation.w = math.cos(yaw / 2)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2) * 1

        self._setpoint_global_pub.publish(
            GeoPoseStamped(
                Header(stamp=rospy.Time.now()),
                pose
            )
        )


    def _send_ned_setpoint(
        self,
        ned_position: NedPosition,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        # purposely not utilizing the ignore yaw feature of sending a local setpoint to force the
        # desired yaw to be continually sent as is required for a global setpoint
        FRAME_LOCAL_NED = 1 # relative to the takeoff position

        self._setpoint_local_pub.publish(
            PositionTarget(
                header=Header(stamp=rospy.Time.now()),
                coordinate_frame=FRAME_LOCAL_NED,
                type_mask=0b111_111_111_000, # zero the position bits
                position=Point(
                    x=ned_position.north,
                    y=ned_position.east,
                    z=ned_position.down
                ),
                yaw=yaw #TODO: does this need to be negative since we are in NED?
            )
        )


    def _send_ned_velocity_setpoint(
        self,
        ned_velocity: NedVelocity,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        # purposely not utilizing the ignore yaw feature of sending a local setpoint to force the
        # desired yaw to be continually sent as is required for a global setpoint
        FRAME_LOCAL_NED = 1 # relative to the takeoff position

        self._setpoint_local_pub.publish(
            PositionTarget(
                header=Header(stamp=rospy.Time.now()),
                coordinate_frame=FRAME_LOCAL_NED,
                type_mask=0b111_111_000_111, # zero the position bits
                velocity=Vector3(
                    x=ned_velocity.north,
                    y=ned_velocity.east,
                    z=ned_velocity.down
                ),
                yaw=yaw #TODO: does this need to be negative since we are in NED?
            )
        )


    def _set_fcu_mode(self, mode: FCUCopterMode) -> bool:
        mode_map: Dict[int, str] = {}
        for internal_mode, px4_mode in self.FCU_MODE_MAP:
            mode_map[internal_mode.value] = px4_mode

        try:
            response: SetModeResponse = self._set_mode_sp(SetModeRequest(
                base_mode=None, # use default
                custom_mode=mode_map[mode.value]
            ))
        except rospy.ServiceException as service_exception:
            rospy.logerr(f"ArdupilotCopterMavrosDrone - set fcu mode exception: {service_exception}")
            return False

        return response.mode_sent

    def _load_parameters(self) -> CopterParameters:
        """ Load the parameter from the FCU, calculates the values in standard units and returns a CopterParameters object
        """
        # Units we need:
        # default Takeoff altitude is in meters.
        # maximum horizontal acceleration is in m/s^2
        # maximum acceleration upwards is in m/s^2
        # maximum acceleration downward is in m/s^2
        # maximum jerk is in m/s^3
        # maximum yaw rate is in radians per second

        # a few parameters are specified in centimeters
        # convert centimeters to meters with a lambda function
        cm_to_m = lambda x: float(x) / 100

        # takeoff converter func
        def takeoff_converter(v):
            if v is not None:
                v = cm_to_m(v)
            if v > 0.0:
                return v
            # TODO - what should we return if the parameter is not found?
            return 7.0

        # need the identity function for some of the parameters
        identity = lambda x: float(x)

        # need to convert from centidegrees per second to radians per second using lambda
        cdeg_to_rad = lambda x: math.radians(float(x) / 100)

        work = {
            # https://ardupilot.org/copter/docs/parameters.html#pilot-tkoff-alt-pilot-takeoff-altitude
            # units = centimeters
            "takeoff_altitude": ("PILOT_TKOFF_ALT", takeoff_converter),
            
            #https://ardupilot.org/copter/docs/parameters.html#wpnav-accel-waypoint-acceleration
            # units = cm/s^2
            "horizontal_acceleration_limit": ("WPNAV_ACCEL", cm_to_m), 
            
            # https://ardupilot.org/copter/docs/parameters.html#wpnav-accel-z-waypoint-vertical-acceleration
            "upward_acceleration_limit": ("WPNAV_ACCEL_Z", cm_to_m), 
            "downward_acceleration_limit": ("WPNAV_ACCEL_Z", cm_to_m), 
            
            # https://ardupilot.org/copter/docs/parameters.html#wpnav-jerk-waypoint-jerk
            # units = cm/s^3
            "jerk_limit": ("WPNAV_JERK", identity), 

            # https://ardupilot.org/copter/docs/parameters.html#atc-slew-yaw-yaw-target-slew-rate
            # this is in centidegrees per second
            "maximum_yaw_rate": ("ATC_SLEW_YAW", cdeg_to_rad),
            # note another idea for maximum yaw rate is to use PILOT_Y_RATE
            # https://ardupilot.org/copter/docs/parameters.html#pilot-y-rate-pilot-controlled-yaw-rate
            # but ATC_SLEW_YAW seems more appropriate
        }

        result = {}
        for param, (fcu_param_name, converter_func) in work.items():
            v = self.get_fcu_parameter(fcu_param_name)
            v = converter_func(v)
            assert v is not None
            result[param] = v
    
        return CopterParameters(**result)




class Px4CopterMavrosDrone(CopterDrone):
    """Communicates with a Px4 controller via mavros to control a copter style vehicle"""
    # http://wiki.ros.org/mavros/CustomModes
    FCU_MODE_MAP = (
        (FCUCopterMode.ALT_HOLD, "ALTCTL"),
        (FCUCopterMode.LAND, "AUTO.LAND"),
        (FCUCopterMode.LOITER, "AUTO.LOITER"),
        (FCUCopterMode.MISSION, "AUTO.MISSION"),
        (FCUCopterMode.OFFBOARD, "OFFBOARD"),
        (FCUCopterMode.POS_HOLD, "POSCTL"),
        (FCUCopterMode.RTL, "AUTO.RTL"),
        (FCUCopterMode.STABILIZE, "STABILIZED"),
        (FCUCopterMode.TAKEOFF, "AUTO.TAKEOFF")
    )

    def __init__(self, gimbal: Gimbal, mavlink_node: MavlinkNode, uav_name: str):
        super().__init__(gimbal=gimbal, uav_name=uav_name)

        self.mavlink_node = mavlink_node

        self._arm_sp: Optional[CommandBool] = None
        self._param_get_sp: Optional[ParamGet] = None
        self._param_set_sp: Optional[ParamSet] = None
        self._set_mode_sp: Optional[SetMode] = None

        # For setpoint publishers we want a queue_size of 1 because new setpoint messages should
        # override old ones, if they've not been sent.
        # Otherwise the drone could be slower to respond to set points
        # more info http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        self._setpoint_local_pub = rospy.Publisher(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self._setpoint_global_pub = rospy.Publisher(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=1
        )
        self._mavlink_pub = rospy.Publisher(
            "mavlink/to", Mavlink, queue_size=1
        )

        # simulated ground control station heartbeat in case we don't have a GCS connected
        self._gcs_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                _GCS_SYSTEM_ID,
                mavlink2.MAV_COMP_ID_MISSIONPLANNER,
                self._mavlink_pub
            ),
            component_type=MavType.GCS,
            send_frequency=2 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._gcs_hb.mav_state = MavState.ACTIVE
        self._gcs_hb.start()

        # mavlink heartbeat from onboard controller to fcu
        self._controller_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                mavlink_node.system_id,
                mavlink_node.component_id,
                self._mavlink_pub
            ),
            component_type=MavType.ONBOARD_CONTROLLER,
            send_frequency=1 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._controller_hb.mav_state = MavState.ACTIVE
        self._controller_hb.start()


    def arm(self) -> bool:
        try:
            response: CommandBoolResponse = self._arm_sp(CommandBoolRequest(value=True))
        except rospy.ServiceException as service_exception:
            rospy.logfatal(
                f"Px4CopterMavrosDrone - arm exception: {service_exception}"
            )
            return False

        return response.success


    def get_fcu_parameter(self, name: str) -> Optional[Union[float, int]]:
        try:
            response: ParamGetResponse = self._param_get_sp(param_id=name)
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"Px4CopterMavrosDrone - get fcu parameter exception: {service_exception}"
            )
            return
        if not response.success:
            rospy.logwarn(f"we could not read the param: '{name}', response = {response}")
            return

        if response.value.real != 0.0:
            return float(response.value.real)

        return int(response.value.integer)


    def set_fcu_parameter(self, name: str, value: Union[float, int]) -> bool:
        if isinstance(value, float):
            msg = ParamValue(
                integer=0,
                real=value
            )
        else:
            msg = ParamValue(
                integer=value,
                real=0.0
            )

        try:
            response: ParamSetResponse = self._param_set_sp(
                param_id=name,
                value=msg
            )
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"Px4CopterMavrosDrone - set fcu parameter exception: {service_exception}"
            )
            return False

        return response.success


    def start(self):
        rospy.wait_for_service("mavros/cmd/arming", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/get", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/set", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/set_mode", _SERVICE_PROXY_TIMEOUT)

        self._arm_sp: CommandBool = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._param_get_sp: ParamGet = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self._param_set_sp: ParamSet = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self._set_mode_sp: SetMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self._set_disarm_preflight()


    def get_available_fcu_modes(self) -> Tuple[FCUCopterMode]:
        return (
            FCUCopterMode.ALT_HOLD,
            FCUCopterMode.LAND,
            FCUCopterMode.LOITER,
            FCUCopterMode.MISSION,
            FCUCopterMode.OFFBOARD,
            FCUCopterMode.POS_HOLD,
            FCUCopterMode.RTL,
            FCUCopterMode.STABILIZE,
            FCUCopterMode.TAKEOFF
        )


    def _set_disarm_preflight(self, disarm_time: float=300.0):
        """Sets the threhold for disarming the drone after arming without a subsequent transition
        Disarm time in seconds with negative values setting indefinite limit
        """
        for _ in range(20):
            disarm_result = self.set_fcu_parameter("COM_DISARM_PRFLT", disarm_time)
            if disarm_result:
                msg = f"Disarm with no transition after arming set to {disarm_time} seconds"
                rospy.loginfo(msg)
                break

            msg = "Unable to set disarm preflight threshold"
            rospy.logerr(msg)
            time.sleep(1)


    def _send_lla_setpoint(self, lla: LlaPosition, yaw: EnuYaw=EnuYaw(0.0)):
        # MAVROS interprets altitude as above mean seal level (AMSL).
        lla = lla.to_amsl()

        pose = GeoPose()
        pose.position.latitude = lla.latitude
        pose.position.longitude = lla.longitude
        pose.position.altitude = lla.altitude

        pose.orientation.w = math.cos(yaw / 2)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2) * 1

        self._setpoint_global_pub.publish(
            GeoPoseStamped(
                Header(stamp=rospy.Time.now()),
                pose
            )
        )


    def _send_ned_setpoint(self, ned_position: NedPosition, yaw: EnuYaw=EnuYaw(0.0)):
        # purposely not utilizing the ignore yaw feature of sending a local setpoint to force the
        # desired yaw to be continually sent as is required for a global setpoint
        FRAME_LOCAL_NED = 1 # relative to the takeoff position

        self._setpoint_local_pub.publish(
            PositionTarget(
                header=Header(stamp=rospy.Time.now()),
                coordinate_frame=FRAME_LOCAL_NED,
                type_mask=0b111_111_111_000, # zero the position bits
                position=Point(
                    x=ned_position.north,
                    y=ned_position.east,
                    z=ned_position.down
                ),
                yaw=yaw #TODO: does this need to be negative since we are in NED?
            )
        )


    def _send_ned_velocity_setpoint(
            self,
            ned_velocity: NedVelocity,
            yaw: EnuYaw=EnuYaw(0.0)
        ):
        # purposely not utilizing the ignore yaw feature of sending a local setpoint to force the
        # desired yaw to be continually sent as is required for a global setpoint
        FRAME_LOCAL_NED = 1 # relative to the takeoff position

        self._setpoint_local_pub.publish(
            PositionTarget(
                header=Header(stamp=rospy.Time.now()),
                coordinate_frame=FRAME_LOCAL_NED,
                type_mask=0b111_111_000_111, # zero the position bits
                velocity=Vector3(
                    x=ned_velocity.north,
                    y=ned_velocity.east,
                    z=ned_velocity.down
                ),
                yaw=yaw #TODO: does this need to be negative since we are in NED?
            )
        )


    def _set_fcu_mode(self, mode: FCUCopterMode) -> bool:
        mode_map: Dict[int, str] = {}
        for internal_mode, px4_mode in self.FCU_MODE_MAP:
            mode_map[internal_mode.value] = px4_mode

        try:
            response: SetModeResponse = self._set_mode_sp(SetModeRequest(
                base_mode=None, # use default
                custom_mode=mode_map[mode.value]
            ))
        except rospy.ServiceException as service_exception:
            rospy.logerr(f"Px4CopterMavrosDrone - set fcu mode exception: {service_exception}")
            return False

        return response.mode_sent

    def _load_parameters(self) -> CopterParameters:
        """ Load the parameter from the FCU, calculates the values in standard units and returns a CopterParameters object
        """
        # Units we need to use
        # default Takeoff altitude is in meters.
        # maximum horizontal acceleration is in m/s^2
        # maximum acceleration upwards is in m/s^2
        # maximum acceleration downward is in m/s^2
        # maximum jerk is in m/s^3
        # maximum yaw rate is in radians per second
        
        def find_value(param, coverter_func):
            v = self.get_fcu_parameter(param)
            return coverter_func(v)
        
        # for PX4 our converter func will be The identity function as a lambda 
        identity = lambda x: x

        param_work = {
            "takeoff_altitude": ("MIS_TAKEOFF_ALT", identity),
            "horizontal_acceleration_limit": ("MPC_ACC_HOR", identity),
            "upward_acceleration_limit": ("MPC_ACC_UP_MAX", identity),
            "downward_acceleration_limit": ("MPC_ACC_DOWN_MAX", identity),
            "jerk_limit": ("MPC_JERK_AUTO", identity),
            "maximum_yaw_rate": ("MPC_YAWRAUTO_MAX", identity),
        }

        result = {}

        for param, (fcu_param_name, converter_func) in param_work.items():
            result[param] = find_value(fcu_param_name, converter_func)
    
        return CopterParameters(**result)
