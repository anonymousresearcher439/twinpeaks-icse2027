import os
import pathlib
import time
import threading
from typing import Any, Dict, List, Tuple, Union, Type
from unittest.mock import Mock, NonCallableMock, patch

import numpy as np

from dr_onboard_autonomy.airlease.protocol import AirLeaserModel, make_model_and_machine
from dr_onboard_autonomy.models.drone import CopterParameters
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.air_lease_service import AirLeaseService
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.states import BaseState, ReadMessages
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.state_factory import STANDARD_TRANSITIONS_FLYING, StateSpec
from dr_onboard_autonomy.mission_helper import MissionBuilder
from dr_onboard_autonomy.gimbal.geolocation import ThreeAxisGimbalQuatCalculator
from dr_onboard_autonomy.models.config import CameraConfig
from dr_onboard_autonomy.models import (
    DATUM_REFERENCE,
    DRConfig,
    CopterDrone,
    FCUCopterMode,
    FCULandedStatus,
    FCUState,
    FCUStatus,
    Gimbal,
    GpsPositionMessage,
)


def read_mission_file(file_name)-> str:
    """Reads a mission file and returns it as a string
    The path is constructued relative to the current file.
    We expect that the mission file is in the ./data directory
    """
    # Get the directory of the current file
    dir_path = os.path.dirname(os.path.realpath(__file__))    

    # Construct the path to the JSON file
    json_file_path = os.path.join(dir_path, 'data', file_name)

    # Open the JSON file and read the data
    with open(json_file_path, 'r') as f:
        return f.read()


def find_config_file_path(file_name)-> pathlib.Path:
    """Reads a config file and returns it as a string

    The file_name should be relative to: test/config/

    For example, to read: test/config/test_configs/test_ardu_mio.toml

    You would call: read_config_file("test_configs/test_ardu_mio.toml")
    """

    test_dir = pathlib.Path(__file__).parent.absolute()
    config_path = test_dir / "config" / file_name
    return config_path


def mock_message_senders():
    message_senders = {}

    def find(name):
        nonlocal message_senders
        if name in message_senders:
            return message_senders[name]
        msg_sender = NonCallableMock(spec=AbstractMessageSender, name=name)
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


def mock_copter_drone():
    drone_params = CopterParameters(
        takeoff_altitude=7.0,
        horizontal_acceleration_limit=3.0,
        upward_acceleration_limit=4.0,
        downward_acceleration_limit=3.0,
        jerk_limit=4.0,
        maximum_yaw_rate=45.0,
    )

    load_params_func = Mock()
    load_params_func.return_value = drone_params

    class TestCopterDrone(CopterDrone):
        def __init__(self, gimbal, uav_name):
            super().__init__(gimbal, uav_name)

        arm = Mock()
        get_fcu_parameter = Mock()
        set_fcu_parameter = Mock()
        start = Mock()
        _send_lla_setpoint = Mock()
        _send_ned_setpoint = Mock()
        _send_ned_velocity_setpoint = Mock()
        get_available_fcu_modes = Mock()
        _set_fcu_mode = Mock()
        _load_parameters = load_params_func

    with patch("dr_onboard_autonomy.models.gimbal.Event", spec=threading.Event):
        with patch("dr_onboard_autonomy.models.gimbal.Thread", spec=threading.Thread):
            class TestGimbal(Gimbal):
                def __init__(self):
                    super().__init__()
                _on_exit = Mock()
                _process_command_queue = Mock()
                _process_messages = Mock()
                receive_message_callback = Mock()
                get_gimbal_status = Mock()
                reaquire_control = Mock()
                set_attitude = Mock()
                set_neutral_attitude = Mock()

            test_gimbal = TestGimbal()

    return TestCopterDrone(test_gimbal, "test_drone")


def mock_mqtt_client(uav_name="test_drone", broker_address="mqtt_broker", broker_port=1883, client_id="", local_ip="0.0.0.0"):
    result = NonCallableMock(
        spec=MQTTClient,
        uav_name=uav_name,
        broker_address=broker_address,
        broker_port=broker_port,
        client_id=client_id,
        local_ip=local_ip,
        client=NonCallableMock(spec=mqtt.Client),
    )
    result.is_connected.return_value = True
    result.connected_event = NonCallableMock(spec=threading.Event)
    result.connected_event.wait.return_value = True
    result.timestamp.return_value = '2024-02-08T02:15:55.512+00:00'
    
    return result


def mock_base_state(customizations:Dict={}):    
    args = dict(
        outcomes=["success"],
        air_lease_protocol_fsm=NonCallableMock(spec=AirLeaseService),
        drone=mock_copter_drone(),
        heartbeat_handler=False,
        local_mqtt_client=NonCallableMock(spec=MQTTClient),
        mqtt_client=NonCallableMock(spec=MQTTClient),
        name="test_state",
        reusable_message_senders=mock_message_senders(),
        trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
    )
    args.update(customizations)
    state = BaseState(**args)
    return state


def mock_state_kwargs(customizations:Dict={}) -> Dict:
    uav_name="test_drone"
    if "uav_id" in customizations:
        uav_name = customizations["uav_id"]
    
    mqtt_gcs = mock_mqtt_client(
        uav_name=uav_name,
        broker_address="mqtt",
        local_ip="1.2.3.4"
    )
    mqtt_local = mock_mqtt_client(
        uav_name=uav_name,
        broker_address="127.0.0.1",
        local_ip="127.0.0.1",
    )

    home = BriarLla.from_args(41.6066954, -86.3555018, 229.27, is_amsl=True)
    result = dict(
        air_lease_protocol_fsm=mock_AirLeaserModel(uav_name, mqtt_gcs),
        data={},
        drone=mock_copter_drone(),
        heartbeat_handler=False,
        mqtt_client=mqtt_gcs,
        local_mqtt_client=mqtt_local,
        reusable_message_senders=mock_message_senders(),
        trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
        home=home.amsl.dict,
        home_altitude_offset=home.amsl.lla.altitude,
        uav_id="test_drone",
        mission_builder=NonCallableMock(spec=MissionBuilder),
        performance_analysis=False,
        gimbal_calculator=ThreeAxisGimbalQuatCalculator(),
        camera_config=CameraConfig(horizontal_fov=74.0, vertical_fov=42.0)
    )
    result.update(customizations)
    return result

def mock_mission_builder_args(customizations:Dict={}):
    mock_args = mock_state_kwargs()
    del mock_args["trajectory_class"]
    del mock_args["home_altitude_offset"]
    del mock_args["mission_builder"]
    del mock_args["gimbal_calculator"]
    del mock_args["camera_config"]
    mock_args.update({
        "state_factory": mock_state_factory(),
        "home": BriarLla.from_args(41.6066954, -86.3555018, 229.27, is_amsl=True),
        "drone_config": mock_drone_config(),
    })
    mock_args.update(customizations)
    return mock_args


def mock_drone_config(
    name="test_drone",
    system_id=1,
    mqtt_host="mqtt",
    mqtt_port=1883,
    gimbal_pitch=True,
    gimbal_roll=True,
    gimbal_yaw=False,
    gimbal_type="gremsy",
    camera_horizontal_fov=74.0,
    camera_vertical_fov=42.0,
    fcu_type="ardupilot"
) -> DRConfig:
    dr_config = DRConfig(**{
        "name": name,
        "system_id": system_id,
        "mqtt": {
            "host": mqtt_host,
            "port": mqtt_port
        },
        "mqtt_local": {
            "host": "mqtt_local",
            "port": mqtt_port
        },
        "gimbal": {
            "pitch": gimbal_pitch,
            "roll": gimbal_roll,
            "yaw": gimbal_yaw,
            "type": gimbal_type
        },
        "fcu": {
            "type": fcu_type
        },
        "camera": {
            "horizontal_fov": camera_horizontal_fov,
            "vertical_fov": camera_vertical_fov
        }
    })

    return dr_config


def mock_mission_builder(customizations:Dict={}):
    return MissionBuilder(**mock_mission_builder_args(customizations))


def mock_state(outcome: str, StateClass: Type[BaseState], customizations:Dict={}):
    """Use this when you want to mock a state that will return a specific
    outcome when you call execute() on it.
    """
    mission_builder = mock_mission_builder()
    kwargs = mission_builder.build_kwargs(customizations)
    state = StateClass(**kwargs)
    
    result = NonCallableMock(wraps=state)
    result.execute.return_value = outcome
    
    return result


def mock_state_factory(customizations:Dict={}):
    def factory_func(state_name: str, transition_spec: List, args: Dict, class_name: str=None) -> StateSpec:
        tx = {}
        for transition in transition_spec:
            tx[transition["condition"]] = transition["target"]
        all_outcomes = set()
        for outcome in tx:
            all_outcomes.add(outcome)
        
        if "outcomes" in args:
            for outcome in args['outcomes']:
                all_outcomes.add(outcome)
        args['outcomes'] = list(all_outcomes)
        mock_state = BaseState(**args)
        return StateSpec(label=state_name, state=mock_state, transitions=tx)

    mock_state_factory = Mock()
    mock_state_factory.side_effect = factory_func
    return mock_state_factory


def mock_state_factory2(extra_states:Dict={}):
    def factory_func(state_name: str, transition_spec: List, args: Dict, class_name: str=None) -> StateSpec:
        from dr_onboard_autonomy.states import FlyWaypoints

        tx = {}
        for outcome in STANDARD_TRANSITIONS_FLYING.keys():
            tx[outcome] = "failure"
    
        for transition in transition_spec:
            tx[transition["condition"]] = transition["target"]
        all_outcomes = set()
        for outcome in tx:
            all_outcomes.add(outcome)
        
        if "outcomes" in args:
            for outcome in args['outcomes']:
                all_outcomes.add(outcome)
        args['outcomes'] = list(all_outcomes)
        if class_name in extra_states:
            StateClass = extra_states[class_name]
            mock_state = StateClass(**args)
        if class_name == "FlyWaypoints":
            mock_state = FlyWaypoints(**args)
        else:
            mock_state = BaseState(**args)
        return StateSpec(label=state_name, state=mock_state, transitions=tx)

    mock_state_factory = Mock()
    mock_state_factory.side_effect = factory_func
    return mock_state_factory


def mock_message(type: str, data: Any):
    return {
        "type": type,
        "data": data
    }

def mock_nav_sat_fix_value(lat, lon, alt, msg_time=None):
    if msg_time is None:
        msg_time = time.time()
    nav_sat = NonCallableMock(spec=NavSatFix)
    nav_sat.status.status = 0 # 0 means we have a fix
    nav_sat.status.service = 1 | 2 | 8 # means we have data from US GPS, GLONASS, and Galileo
    nav_sat.header.stamp.to_sec.side_effect = lambda: msg_time
    nav_sat.header.stamp.to_time.side_effect = lambda: msg_time
    nav_sat.latitude = lat
    nav_sat.longitude = lon
    nav_sat.altitude = alt
    nav_sat.position_covariance_type = 3
    return nav_sat


def mock_message_sender_position_message(
        lat: float,
        lon: float,
        alt: float,
        alt_ref=DATUM_REFERENCE.ELLIPSOID_WGS84,
        message_created_time: float=time.time(),
        gps_fix=True):
    """Fixture for an internal message sender position
    """
    return mock_message(
        "position",
        GpsPositionMessage(
            latitude=lat,
            longitude=lon,
            altitude=alt,
            datum_ref=alt_ref,
            time=message_created_time,
            is_fix=gps_fix
        )
    )


def mock_position(lat, lon, alt, msg_time=None):
    nav_sat = mock_nav_sat_fix_value(lat, lon, alt, msg_time)
    
    import dr_onboard_autonomy.message_converters as mc
    return mc.ros_position_adapter(nav_sat)


def mock_position_message(lat, lon, alt, msg_time=None):
    pos = mock_position(lat, lon, alt, msg_time)
    return mock_message("position", pos)


def unit_quaternion(x,y,z, theta) -> Tuple[float, float, float, float]:
    # we need to make x,y,z a unit vector
    norm = (x**2 + y**2 + z**2)**0.5
    assert norm > 0, "x, y, z must not be all zero"
    x /= norm
    y /= norm
    z /= norm

    
    half_theta = theta / 2.0
    sin_half_theta = np.sin(half_theta)
    return (
        x * sin_half_theta,
        y * sin_half_theta,
        z * sin_half_theta,
        np.cos(half_theta),
    )


def mock_ros_imu_data(x, y, z, w, msg_time=None):
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

def mock_imu_data(x, y, z, w, t=None):
    ros_imu = mock_ros_imu_data(x, y, z, w, t)
    from dr_onboard_autonomy.message_converters import ros_imu_adapter
    return ros_imu_adapter(ros_imu)


def mock_message_sender_state_message(
    armed: bool=True,
    connected: bool=True,
    mode: FCUCopterMode=FCUCopterMode.OFFBOARD,
    status: FCUStatus=FCUStatus.ACTIVE,
    landed_status: FCULandedStatus=FCULandedStatus.IN_AIR
):
    return mock_message("state", FCUState(
        armed=armed,
        connected=connected,
        mode=mode,
        status=status,
        landed_status=landed_status
    ))


def mock_state_message(connected:bool=True, armed:bool=True, guided:bool=True, manual_input:bool=False, mode:str="OFFBOARD", system_status:int=3, msg_time:float=None):
    """create a state message like the kind that you can view with rostopic echo mavros/state
    possible modes:
        - "OFFBOARD"
        - "AUTO.TAKEOFF"
        - "AUTO.LAND"
        - "STABILIZED"
        - "AUTO.LOITER"
        - "AUTO.RTL"
        - "POSCTL"
        - "ACRO"
        - "ALTCTL"
        - "AUTO.MISSION"
        - "AUTO.READY"
        - "AUTO.RTGS"
        - "MANUAL"
        - "RATTITUDE"

    """
    if msg_time is None:
        msg_time = time.time()
    state_msg = NonCallableMock(spec=State)
    state_msg.header.stamp.to_sec.side_effect = lambda: msg_time
    state_msg.header.stamp.to_time.side_effect = lambda: msg_time
    state_msg.connected = connected
    state_msg.armed = armed
    state_msg.guided = guided
    state_msg.manual_input = manual_input
    state_msg.mode = mode
    state_msg.system_status = system_status

    return mock_message("state", state_msg)


def mock_shutdown_message():
    return {"type": "shutdown", "data": None}


def mock_mqtt_message(msg_sender_name: str, topic: Union[bytes, str], payload: Union[bytes, str], mid=None, qos=0, retain=False, timestamp=None):
    # from paho.mqtt.client import MQTTMessage
    
    if isinstance(topic, str):
        # make bytes
        topic = topic.encode("utf-8")
    
    if isinstance(payload, str):
        # make bytes
        payload = payload.encode("utf-8")
    
    if mid is None:
        mid = 1
    
    if timestamp is None:
        timestamp = 358311.916657896
    
    mqtt_message = MQTTMessage(mid, topic)
    mqtt_message.payload = payload
    mqtt_message.qos = qos
    mqtt_message.retain = retain
    mqtt_message.timestamp = timestamp

    return mock_message(msg_sender_name, mqtt_message)


def mock_ros_battery_data(voltage, current, percentage) -> BatteryState:
    battery = BatteryState()
    battery.voltage = voltage
    battery.current = current
    battery.percentage = percentage
    return battery


def mock_battery_data(voltage, current, percentage):
    ros_data = mock_ros_battery_data(voltage, current, percentage)
    from dr_onboard_autonomy.message_converters import ros_battery_adapter
    return ros_battery_adapter(ros_data)


def mock_compass_hdg_data(compass_heading: float) -> Float64:
    return Float64(data=compass_heading)


def mock_relative_altitude_data(relative_altitude: float) -> Float64:
    return Float64(data=relative_altitude)

def mock_velocity_data(linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
    result = TwistStamped()
    result.twist.linear.x = linear_x
    result.twist.linear.y = linear_y
    result.twist.linear.z = linear_z
    result.twist.angular.x = angular_x
    result.twist.angular.y = angular_y
    result.twist.angular.z = angular_z
    return result


def mock_ReadMessages(current_pos: BriarLla):
    """
    This creates a mock ReadMessages instance that will return "succeeded" when you call execute().
    It will make a a dictionary with a "position" key where the value is a ROS NavSatFix message when you call get()
    """
    mock_ReadMessages_instance = NonCallableMock(spec=ReadMessages)
    # we need the execute method to return "succeeded"
    mock_ReadMessages_instance.execute.return_value = "succeeded"
    # we need the get method to return a dictionary with a "position" key
    # where the value is a ROS NavSatFix message
    mock_ReadMessages_instance.get.return_value = {
        "position" : mock_nav_sat_fix_value(*current_pos.ellipsoid.tup)
    }
    return mock_ReadMessages_instance


def mock_AirLeaseService(mqtt_client):
    instance = AirLeaseService("test_drone", mqtt_client)
    # need to wrap this real instance in a mock so that we override the return values
    # and verify usage
    mock_instance = NonCallableMock(wraps=instance)
    return mock_instance

def mock_AirLeaserModel(uav_name, mqtt_client):
    # instance = AirLeaserModel(uav_name, mqtt_client)
    model, machine = make_model_and_machine(uav_name, mqtt_client)
    return NonCallableMock(wraps=model)


def mock_ROS_ExtendedState(landed_state=2):
    """Create aROS ExtendedState message
    set the vtol_state to VTOL_STATE_UNDEFINED
    set the landed_state to the value of landed_state. 

    The landed_state must be one of the following:
        LANDED_STATE_UNDEFINED=0
        LANDED_STATE_ON_GROUND=1
        LANDED_STATE_IN_AIR=2
        LANDED_STATE_TAKEOFF=3
        LANDED_STATE_LANDING=4
    """
    VTOL_STATE_UNDEFINED=0
    VTOL_STATE_TRANSITION_TO_FW=1
    VTOL_STATE_TRANSITION_TO_MC=2
    VTOL_STATE_MC=3
    VTOL_STATE_FW=4
    LANDED_STATE_UNDEFINED=0
    LANDED_STATE_ON_GROUND=1
    LANDED_STATE_IN_AIR=2
    LANDED_STATE_TAKEOFF=3
    LANDED_STATE_LANDING=4
    ext_state = ExtendedState()
    ext_state.vtol_state = VTOL_STATE_UNDEFINED
    ext_state.landed_state = landed_state
    
    return ext_state


import unittest


class TestMockTypes(unittest.TestCase):

    def test_mock_ReadMessages(self):
        # here is the standard usage scenario for ReadMessages
        # define the drone's current position
        current_pos = BriarLla.from_args(41.6066954, -86.3555018, 229.27, is_amsl=True)
        # patch the ReadMessages class in a given module and get a reference to it
        # in this case we will just create a Mock object as a standin for this step
        mock_ReadMessages_class = Mock()
        # set the return value of the mock class to be a mock instance of ReadMessages
        mock_ReadMessages_class.return_value = mock_ReadMessages(current_pos)

        # Now let's make sure it works
        x = mock_ReadMessages_class()
        self.assertEqual(x.execute(None), "succeeded")
        data = x.get()
        self.assertIn("position", data)
        self.assertAlmostEqual(data["position"].latitude, current_pos.ellipsoid.lla.lat)
        self.assertAlmostEqual(data["position"].longitude, current_pos.ellipsoid.lla.lon)
        self.assertAlmostEqual(data["position"].altitude, current_pos.ellipsoid.lla.alt)
        self.assertEqual(len(data), 1)
        self.assertIsInstance(data["position"], NavSatFix)