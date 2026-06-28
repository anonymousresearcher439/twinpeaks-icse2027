import math
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
    ParamGetRequest,
    ParamGetResponse,
    ParamSet,
    ParamSetRequest,
    ParamSetResponse,
    SetMode,
    SetModeRequest,
    SetModeResponse
)
from pymavlink.dialects.v20 import common as mavlink2
import rospy

from dr_onboard_autonomy.models import (
    CopterDrone,
    EnuYaw,
    FCUCopterMode,
    Gimbal,
    LlaPosition,
    NedPosition,
    NedVelocity
)
from dr_onboard_autonomy.mavlink import HeartbeatSender, MavlinkNode, MavlinkSender

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
        {"msg_id": mavlink2.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, "frequency": 1}
    )


    def __init__(self, gimbal: Gimbal, mavlink_node: MavlinkNode, uav_name: str):
        super().__init__(gimbal=gimbal, uav_name=uav_name)

        self.mavlink_node = mavlink_node

        rospy.wait_for_service("mavros/cmd/arming", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/get", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/set", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/set_mode", _SERVICE_PROXY_TIMEOUT)

        self._arm_sp: CommandBool = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._param_get_sp: ParamGet = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self._param_set_sp: ParamSet = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self._set_mode_sp: SetMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

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
            component_type=mavlink2.MAV_TYPE_GCS,
            send_frequency=2 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._gcs_hb.mav_state = mavlink2.MAV_STATE_ACTIVE
        self._gcs_hb.start()

        # mavlink heartbeat from onboard controller to fcu
        self._controller_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                mavlink_node.system_id,
                mavlink_node.component_id,
                self._mavlink_pub
            ),
            component_type=mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
            send_frequency=1 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._controller_hb.mav_state = mavlink2.MAV_STATE_ACTIVE
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
            response: ParamGetResponse = self._param_get_sp(ParamGetRequest(param_id=name))
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
            response: ParamSetResponse = self._param_set_sp(ParamSetRequest(
                param_id=name,
                value=msg
            ))
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"ArdupilotCopterMavrosDrone - set fcu parameter exception: {service_exception}"
            )
            return False

        return response.success


    def start(self):
        self._request_mavlink_messages()


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
        lla.to_amsl()

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

        rospy.wait_for_service("mavros/cmd/arming", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/get", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/param/set", _SERVICE_PROXY_TIMEOUT)
        rospy.wait_for_service("mavros/set_mode", _SERVICE_PROXY_TIMEOUT)

        self._arm_sp: CommandBool = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._param_get_sp: ParamGet = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self._param_set_sp: ParamSet = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self._set_mode_sp: SetMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

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
            component_type=mavlink2.MAV_TYPE_GCS,
            send_frequency=2 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._gcs_hb.mav_state = mavlink2.MAV_STATE_ACTIVE
        self._gcs_hb.start()

        # mavlink heartbeat from onboard controller to fcu
        self._controller_hb = HeartbeatSender(
            mavlink_sender=MavlinkSender(
                mavlink_node.system_id,
                mavlink_node.component_id,
                self._mavlink_pub
            ),
            component_type=mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
            send_frequency=1 # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
        )
        self._controller_hb.mav_state = mavlink2.MAV_STATE_ACTIVE
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
            response: ParamGetResponse = self._param_get_sp(ParamGetRequest(param_id=name))
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"Px4CopterMavrosDrone - get fcu parameter exception: {service_exception}"
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
            response: ParamSetResponse = self._param_set_sp(ParamSetRequest(
                param_id=name,
                value=msg
            ))
        except rospy.ServiceException as service_exception:
            rospy.logerr(
                f"Px4CopterMavrosDrone - set fcu parameter exception: {service_exception}"
            )
            return False

        return response.success


    def start(self):
        pass


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


    def _send_lla_setpoint(self, lla: LlaPosition, yaw: EnuYaw=EnuYaw(0.0)):
        # MAVROS interprets altitude as above mean seal level (AMSL).
        lla.to_amsl()

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
