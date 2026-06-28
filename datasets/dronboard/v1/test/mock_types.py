import time
from typing import Any
from unittest.mock import Mock, NonCallableMock, patch

from std_msgs.msg import Header, Time
from sensor_msgs.msg import NavSatFix

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator

def mock_message_senders():
    message_senders = {}

    def find(name):
        nonlocal message_senders
        if name in message_senders:
            return message_senders[name]
        msg_sender = NonCallableMock(spec=AbstractMessageSender)
        msg_sender.name = name
        message_senders[name] = msg_sender
        return msg_sender

    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.find.side_effect = find
    return reusable_message_senders

def mock_drone():
    with patch("dr_onboard_autonomy.mavros_layer.rospy") as rospy_mock:
        drone = MAVROSDrone(uav_name="test_drone", simulate_gcs_heartbeat=False, init_gimbal=True)

    # drone._param_set_sp.side_effect = mock_param_set
    mock_setpoint_msg = Mock()
    mock_setpoint_msg.type_mask = 0

    drone._build_local_setpoint = Mock()
    drone._build_local_setpoint.return_value = mock_setpoint_msg

    drone._build_global_setpoint2 = Mock()
    drone._build_global_setpoint2.return_value = mock_setpoint_msg

    drone._build_global_setpoint3 = Mock()
    drone._build_global_setpoint3.return_value = mock_setpoint_msg

    return drone

def mock_base_state(outcomes=["success"]):
    state = BaseState(
        outcomes=outcomes,
        air_lease_service=NonCallableMock(spec=AirLeaseService),
        drone=NonCallableMock(spec=MAVROSDrone),
        heartbeat_handler=False,
        local_mqtt_client=NonCallableMock(spec=MQTTClient),
        mqtt_client=NonCallableMock(spec=MQTTClient),
        name="test_state",
        reusable_message_senders=mock_message_senders(),
        trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
    )
    return state


def mock_message(type: str, data: Any):
    return {
        "type": type,
        "data": data
    }

def mock_position_message(lat, lon, alt, msg_time=None):
    if msg_time is None:
        msg_time = time.time()
    nav_sat = NonCallableMock()
    nav_sat.status.status = 0 # 0 means we have a fix
    nav_sat.status.service = 1 | 2 | 8 # means we have data from US GPS, GLONASS, and Galileo
    nav_sat.header.stamp.to_sec.side_effect = lambda: msg_time
    nav_sat.header.stamp.to_time.side_effect = lambda: msg_time
    nav_sat.latitude = lat
    nav_sat.longitude = lon
    nav_sat.altitude = alt
    nav_sat.position_covariance_type = 3
    return mock_message("position", nav_sat)

def mock_imu_message(x, y, z, w, msg_time=None):
    if msg_time is None:
        msg_time = time.time()
    imu = NonCallableMock()
    imu.header.stamp.to_sec.side_effect = lambda: msg_time
    imu.header.stamp.to_time.side_effect = lambda: msg_time

    imu.orientation.x = x
    imu.orientation.y = y
    imu.orientation.z = z
    imu.orientation.w = w

    imu.angular_velocity.x = 0
    imu.angular_velocity.y = 0
    imu.angular_velocity.z = 0

    imu.linear_acceleration.x = 0
    imu.linear_acceleration.y = 0
    imu.linear_acceleration.z = 0
    return imu