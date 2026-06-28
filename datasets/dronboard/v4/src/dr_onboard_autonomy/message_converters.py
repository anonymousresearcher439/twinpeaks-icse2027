from std_msgs.msg import (
    Float64,
    String,
)
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import (
    BatteryState,
    NavSatFix,
    TimeReference,
)
from sensor_msgs.msg import Imu as ImuRos
from mavros_msgs.msg import (
    EstimatorStatus,
    ExtendedState,
    RCIn,
    State,
    GPSRAW,
    AttitudeTarget,
    Vibration as RosVibration,
    RCOut as RosRCOut,    
)
import numpy as np


from dr_onboard_autonomy.models import (
    Battery,
    FCUState,
    FCULandedStatus,
    FCUStatus,
    Quaternion,
    LlaPosition,
    DATUM_REFERENCE,
    NedAcceleration,
    NedVelocity,
    NedPosition,
    IMU,
    HeartbeatStatus,
    TimeStampedLlaPosition,
    TimeStampedQuaternion,
    GpsPositionMessage,
    RelativeAltitude,
    EnuYaw,
    TimeRef,
    TimeSource,
    GPSStatus,
    Vibration,
    RCOut,
    EstimatorStatusData
)

from dr_onboard_autonomy.models import FCUCopterMode 

# ("battery", "mavros/battery", BatteryState),
# ("state", "mavros/state", State),
# ("imu", "mavros/imu/data", Imu),
# ("compass_hdg", "mavros/global_position/compass_hdg", Float64),
# ("velocity", "mavros/global_position/raw/gps_vel", TwistStamped),
# ("position", "mavros/global_position/global", NavSatFix),
# ("relative_altitude", "mavros/global_position/rel_alt", Float64),


# ("extended_state", "mavros/extended_state", ExtendedState),
# ("estimator_status", "mavros/estimator_status", EstimatorStatus),
# ("rcin", "mavros/rc/in", RCIn),
# ("diagnostics", "/diagnostics", DiagnosticArray),


def ros_battery_adapter(ros_data: BatteryState) -> Battery:
    return Battery(
        voltage=ros_data.voltage,
        current=ros_data.current,
        level=ros_data.percentage
    )

# TODO should this be used in the IMU message?
def ros_attitude_adapter(ros_data: ImuRos) -> TimeStampedQuaternion:
    attitude = Quaternion(
        x=ros_data.orientation.x,
        y=ros_data.orientation.y,
        z=ros_data.orientation.z,
        w=ros_data.orientation.w
    )
    return TimeStampedQuaternion(
        time=ros_data.header.stamp.to_time(),
        quaternion=attitude
    )


def ros_imu_adapter(ros_data: ImuRos) -> IMU:
    return IMU(
        time=ros_data.header.stamp.to_time(),
        acceleration_linear=NedAcceleration(
            east=ros_data.linear_acceleration.x,
            north=ros_data.linear_acceleration.y,
            down=ros_data.linear_acceleration.z * -1.0,
        ),
        attitude=Quaternion(
            x=ros_data.orientation.x,
            y=ros_data.orientation.y,
            z=ros_data.orientation.z,
            w=ros_data.orientation.w
        )
    )


def ros_compass_heading_adapter(ros_data: Float64) -> EnuYaw:
    return EnuYaw(ros_data.data)


def ros_ground_speed_adapter(ros_data: TwistStamped) -> float:
    # we need the magnitude of ros_data.twist.linear.x and ros_data.twist.linear.y
    east = ros_data.twist.linear.x
    north = ros_data.twist.linear.y
    ground_speed = np.linalg.norm([east, north])
    return float(ground_speed)


def ros_velocity_adapter(ros_data: TwistStamped) -> NedVelocity:
    """converts a ros velocity message to a NedVelocity object
    """
    # ROS uses ENUs, so we need to swap the x and y values. We also need to negate the z value
    # TODO check that ROS gives us ENU values
    return NedVelocity(
        north=ros_data.twist.linear.y,
        east=ros_data.twist.linear.x,
        down=ros_data.twist.linear.z * -1.0,
    )


def ros_gps_status(ros_data: GPSRAW) -> GPSStatus:
    return GPSStatus(
        satellites_visible=ros_data.satellites_visible,
        hdop=ros_data.eph,
        vdop=ros_data.epv
    )


def ros_vibration_adapter(ros_data: RosVibration) -> Vibration:
    return Vibration(
        x=ros_data.vibration.x,
        y=ros_data.vibration.y,
        z=ros_data.vibration.z,
        clipping0=ros_data.clipping[0],
        clipping1=ros_data.clipping[1],
        clipping2=ros_data.clipping[2]
    )




def ros_rc_out_adapter(ros_data: RosRCOut) -> RCOut:
    return RCOut(channels=tuple(ros_data.channels[0:12]))



from dr_onboard_autonomy.models import GPSStatus, Vibration, RCOut, EstimatorStatusData

def ros_estimator_adapter(ros_data: EstimatorStatus) -> EstimatorStatusData:
    return EstimatorStatusData(
        attitude_status_flag=ros_data.attitude_status_flag,
        velocity_horiz_status_flag=ros_data.velocity_horiz_status_flag,
        velocity_vert_status_flag=ros_data.velocity_vert_status_flag,
        pos_horiz_rel_status_flag=ros_data.pos_horiz_rel_status_flag,
        pos_horiz_abs_status_flag=ros_data.pos_horiz_abs_status_flag,
        pos_vert_abs_status_flag=ros_data.pos_vert_abs_status_flag,
        pos_vert_agl_status_flag=ros_data.pos_vert_agl_status_flag,
        const_pos_mode_status_flag=ros_data.const_pos_mode_status_flag,
        pred_pos_horiz_rel_status_flag=ros_data.pred_pos_horiz_rel_status_flag,
        pred_pos_horiz_abs_status_flag=ros_data.pred_pos_horiz_abs_status_flag,
        gps_glitch_status_flag=ros_data.gps_glitch_status_flag,
        accel_error_status_flag=ros_data.accel_error_status_flag
    )



# Use existing Quaternion dataclass for target attitude
def ros_targ_att_adapter(ros_data: AttitudeTarget) -> Quaternion:
    orientation = ros_data.orientation
    return Quaternion(
        x=orientation.x,
        y=orientation.y,
        z=orientation.z,
        w=orientation.w
    )

def ros_position_adapter(ros_data: NavSatFix) -> TimeStampedLlaPosition:
    lat, lon, alt = ros_data.latitude, ros_data.longitude, ros_data.altitude
    fix_status = ros_data.status.status
    return GpsPositionMessage(
        time=ros_data.header.stamp.to_time(),
        latitude=lat,
        longitude=lon,
        altitude=alt,
        is_fix=fix_status > -1, # https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatStatus.html
        datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84,
    )


def ros_time_reference_adapter(ros_data: TimeReference) -> TimeRef:
    secs = ros_data.time_ref.secs
    nsecs = ros_data.time_ref.nsecs
    total_nanoseconds = secs * 1_000_000_000 + nsecs
    source = TimeSource.UNKNOWN
    if ros_data.source == "fcu":
        source = TimeSource.FCU
    return TimeRef(total_nanoseconds, source)


def ros_relative_altitude_adapter(ros_data: Float64) -> RelativeAltitude:
    return RelativeAltitude(ros_data.data)


def ros_fcu_status_adapter(ros_data: State) -> FCUStatus:
    mav_state = ros_data.system_status

    MAV_STATE_UNINIT = 0
    MAV_STATE_BOOT = 1	
    MAV_STATE_CALIBRATING = 2
    MAV_STATE_STANDBY = 3	
    MAV_STATE_ACTIVE = 4	
    MAV_STATE_CRITICAL = 5	
    MAV_STATE_EMERGENCY = 6	
    MAV_STATE_POWEROFF = 7	
    MAV_STATE_FLIGHT_TERMINATION = 8

    status_mapping = {
        MAV_STATE_UNINIT: FCUStatus.UNKNOWN,
        MAV_STATE_BOOT: FCUStatus.BOOT,
        MAV_STATE_CALIBRATING: FCUStatus.CALIBRATE,
        MAV_STATE_STANDBY: FCUStatus.STANDBY,
        MAV_STATE_ACTIVE: FCUStatus.ACTIVE,
        MAV_STATE_CRITICAL: FCUStatus.CRITICAL,
        MAV_STATE_EMERGENCY: FCUStatus.EMERGENCY,
        MAV_STATE_POWEROFF: FCUStatus.POWER_OFF,
        MAV_STATE_FLIGHT_TERMINATION: FCUStatus.TERMINATE,
    }
    return status_mapping.get(mav_state, FCUStatus.UNKNOWN)


def _map_extended_state(ros_data: ExtendedState) -> FCULandedStatus:
    LANDED_STATE_UNDEFINED=0
    LANDED_STATE_ON_GROUND=1
    LANDED_STATE_IN_AIR=2
    LANDED_STATE_TAKEOFF=3
    LANDED_STATE_LANDING=4

    status_mapping = {
        LANDED_STATE_UNDEFINED: FCULandedStatus.UNKNOWN,
        LANDED_STATE_ON_GROUND: FCULandedStatus.ON_GROUND,
        LANDED_STATE_IN_AIR: FCULandedStatus.IN_AIR,
        LANDED_STATE_TAKEOFF: FCULandedStatus.UNKNOWN,
        LANDED_STATE_LANDING: FCULandedStatus.UNKNOWN,
    }
    return status_mapping.get(ros_data.landed_state, FCULandedStatus.UNKNOWN)

def ros_state_adapter(ros_data: State, ros_extended_state: ExtendedState=None) -> FCUState:
    armed = ros_data.armed
    is_connected = ros_data.connected
    # TODO review this implementation
    mode = _px4_map_mode(ros_data.mode)
    if mode == FCUCopterMode.UNKNOWN:
        mode = _arducopter_map_mode(ros_data.mode)
    status = ros_fcu_status_adapter(ros_data)

    landed_state = FCULandedStatus.UNKNOWN
    if ros_extended_state:
        landed_state = _map_extended_state(ros_extended_state)
    return FCUState(armed, is_connected, mode, status, landed_state)
    

def _px4_map_mode(mode: str) -> FCUCopterMode:
    px4_mode_map = {
        "ACRO": FCUCopterMode.MANUAL_OTHER,
        "ALTCTL": FCUCopterMode.ALT_HOLD, 
        "AUTO.LAND": FCUCopterMode.LAND, 
        "AUTO.LOITER": FCUCopterMode.LOITER,
        "AUTO.MISSION":  FCUCopterMode.MISSION,
        "AUTO.READY": FCUCopterMode.UNKNOWN,
        "AUTO.RTGS": FCUCopterMode.RTL, # TODO double check this. Is this RETURN TO GROUND STATION??? Should this be UNKNOWN????
        "AUTO.RTL": FCUCopterMode.RTL,
        "AUTO.TAKEOFF": FCUCopterMode.TAKEOFF,
        "MANUAL": FCUCopterMode.MANUAL_OTHER, 
        "OFFBOARD": FCUCopterMode.OFFBOARD,
        "POSCTL": FCUCopterMode.POS_HOLD,
        "RATTITUDE": FCUCopterMode.MANUAL_OTHER,
        "STABILIZED": FCUCopterMode.STABILIZE,
    }
    return px4_mode_map.get(mode, FCUCopterMode.UNKNOWN)

def _arducopter_map_mode(mode:str) -> FCUCopterMode:
    arducopter_mode_map = {
        "STABILIZE": FCUCopterMode.STABILIZE,
        "ACRO": FCUCopterMode.MANUAL_OTHER,
        "ALT_HOLD": FCUCopterMode.ALT_HOLD,
        "AUTO": FCUCopterMode.MISSION,
        "GUIDED": FCUCopterMode.OFFBOARD,
        "LOITER": FCUCopterMode.LOITER,
        "RTL": FCUCopterMode.RTL,
        "CIRCLE": FCUCopterMode.UNKNOWN,
        "POSITION": FCUCopterMode.POS_HOLD,
        "LAND": FCUCopterMode.LAND,
        "OF_LOITER": FCUCopterMode.LOITER, # TODO Double check
        "DRIFT": FCUCopterMode.MANUAL_OTHER,
        "SPORT": FCUCopterMode.MANUAL_OTHER,
        "FLIP": FCUCopterMode.MANUAL_OTHER,
        "AUTOTUNE": FCUCopterMode.MANUAL_OTHER, # TODO Double check
        "POSHOLD": FCUCopterMode.POS_HOLD,
        "BRAKE": FCUCopterMode.MANUAL_OTHER, # This is an emergency stop mode... should we use a different value?
        "THROW": FCUCopterMode.MANUAL_OTHER, # This is a special mode for throwing the drone into the air, like a takeoff mode
        "AVOID_ADSB": FCUCopterMode.MANUAL_OTHER, # This is a special mode for avoiding ADSB traffic. It works in conjunction with other modes???
        "GUIDED_NOGPS": FCUCopterMode.MANUAL_OTHER,
    }
    return arducopter_mode_map.get(mode, FCUCopterMode.UNKNOWN)

