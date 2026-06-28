
import importlib
import json
import math
import unittest
from queue import Queue
from unittest.mock import NonCallableMock

from dr_onboard_autonomy.airlease.protocol import make_model_and_machine
from dr_onboard_autonomy.message_converters import ros_imu_adapter
from dr_onboard_autonomy.models.kinematics import Quaternion
from dr_onboard_autonomy.states.components.StatusMessage import StatusMessage
from .mock_types import (
    mock_ros_imu_data,
    mock_state,
    mock_message,
    mock_shutdown_message,
    mock_battery_data,
    mock_message_sender_state_message,
    mock_imu_data,
    mock_compass_hdg_data,
    mock_position,
    mock_base_state,
    unit_quaternion,
)
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.models import FCUCopterMode, FCUStatus, FCULandedStatus, EnuYaw, NedVelocity, RelativeAltitude, LlaPosition
import numpy as np


StatusMessage_module = importlib.import_module(StatusMessage.__module__)


class TestBaseStateDataUpdate(unittest.TestCase):
    def setUp(self):
        self.StateClass = BaseState
        self.userdata = {}

    def test_update_battery(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate with None
        state.drone.data.battery = None
        # Create and send battery message
        battery = mock_battery_data(12.3, 4.5, 67.8)
        state.message_queue.put(mock_message("battery", battery))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        self.assertIsNotNone(state.drone.data.battery)
        self.assertEqual(state.drone.data.battery.voltage, 12.3)
        self.assertEqual(state.drone.data.battery.current, 4.5)
        self.assertEqual(state.drone.data.battery.level, 67.8)

    def test_update_state(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate with different values
        state.drone.data.armed = False
        state.drone.data.connected = False
        state.drone.data.mode = FCUCopterMode.UNKNOWN
        state.drone.data.status = FCUStatus.UNKNOWN
        state.drone.data.landed_status = FCULandedStatus.UNKNOWN
        # Create and send state message
        msg = mock_message_sender_state_message(
            armed=True,
            connected=True,
            mode=FCUCopterMode.OFFBOARD,
            status=FCUStatus.ACTIVE,
            landed_status=FCULandedStatus.IN_AIR
        )
        state.message_queue.put(msg)
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        self.assertTrue(state.drone.data.armed)
        self.assertTrue(state.drone.data.connected)
        self.assertEqual(state.drone.data.mode, FCUCopterMode.OFFBOARD)
        self.assertEqual(state.drone.data.status, FCUStatus.ACTIVE)
        self.assertEqual(state.drone.data.landed_status, FCULandedStatus.IN_AIR)

    def test_update_imu(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate attitude with no data
        # state.drone.data.attitude = state.drone.data.attitude.__class__()  # reset
        assert state.drone is not None
        attitude_0 = Quaternion(0.0, 0.0, 0.0, 1.0)
        state.drone.data.attitude.add_attitude(1.0, attitude_0)
        # Create and send imu message
        x,y,z,w = unit_quaternion(1.0, 1.0, 1.0, math.radians(45))
        ros_imu = mock_ros_imu_data(x,y,z,w, msg_time=2.0)
        
        ros_imu.angular_velocity.x = 1.0
        ros_imu.angular_velocity.y = 2.0
        ros_imu.angular_velocity.z = 3.0
        ros_imu.linear_acceleration.x = 4.0
        ros_imu.linear_acceleration.y = 5.0
        ros_imu.linear_acceleration.z = 6.0
        imu = ros_imu_adapter(ros_imu)

        state.message_queue.put(mock_message("imu", imu))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)

        t0, att0 = state.drone.data.attitude[0]
        self.assertEqual(t0, 1.0)
        self.assertIsNotNone(att0)
        # make sure this matches attitude_0
        self.assertEqual(att0[0], attitude_0.x)
        self.assertEqual(att0[1], attitude_0.y)
        self.assertEqual(att0[2], attitude_0.z)
        self.assertEqual(att0[3], attitude_0.w)

        t1, att1 = state.drone.data.attitude[1]
        self.assertEqual(t1, 2.0)
        self.assertIsNotNone(att1)
        # make sure this matches the imu data
        self.assertAlmostEqual(att1[0], x)
        self.assertAlmostEqual(att1[1], y)
        self.assertAlmostEqual(att1[2], z)
        self.assertAlmostEqual(att1[3], w)

        att = state.drone.data.attitude.get_attitude()
        self.assertIsNotNone(att)
        self.assertEqual(att.x, x)
        self.assertEqual(att.y, y)
        self.assertEqual(att.z, z)
        self.assertEqual(att.w, w)

    def test_update_compass_hdg(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate heading
        state.drone.data.heading = None
        heading = mock_compass_hdg_data(42.0)
        # The update_drone_data expects an EnuYaw, not a Float64, so wrap as EnuYaw
        enu_yaw = EnuYaw(heading.data)
        state.message_queue.put(mock_message("compass_hdg", enu_yaw))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        self.assertIsNotNone(state.drone.data.heading)
        self.assertEqual(state.drone.data.heading, 42.0)

    def test_update_velocity(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate velocity
        state.drone.data.velocity = None
        state.drone.data.ground_speed = None
        # NedVelocity is expected, so create it directly
        velocity = NedVelocity(north=3.0, east=4.0, down=0.0)
        state.message_queue.put(mock_message("velocity", velocity))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        self.assertIsNotNone(state.drone.data.velocity)
        self.assertEqual(getattr(state.drone.data.velocity, 'north', None), 3.0)
        self.assertEqual(getattr(state.drone.data.velocity, 'east', None), 4.0)
        self.assertAlmostEqual(state.drone.data.ground_speed, 5.0)

    def test_update_position(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate location
        state.drone.data.location = state.drone.data.location.__class__()  # reset
        pos = mock_position(10.0, 20.0, 30.0, msg_time=123.45)
        state.message_queue.put(mock_message("position", pos))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        latest = state.drone.data.location.get_position()
        self.assertIsNotNone(latest)
        self.assertAlmostEqual(latest.latitude, 10.0)
        self.assertAlmostEqual(latest.longitude, 20.0)
        self.assertAlmostEqual(latest.altitude, 30.0)

    def test_update_relative_altitude(self):
        state = mock_base_state()
        state.message_queue = Queue()
        # Pre-populate relative_altitude
        assert state.drone
        state.drone.data.relative_altitude = RelativeAltitude(-1.0)
        rel_alt = RelativeAltitude(123.4)
        state.message_queue.put(mock_message("relative_altitude", rel_alt))
        state.message_queue.put(mock_shutdown_message())
        state.execute(self.userdata)
        self.assertIsNotNone(state.drone.data.relative_altitude)
        self.assertEqual(state.drone.data.relative_altitude, 123.4)
"""Tests for the StatusMessage component"""
import sys
import unittest

from unittest.mock import (
    Mock,
    NonCallableMagicMock,
    patch
)
from queue import Queue

from dr_onboard_autonomy.states import BaseState

from .mock_types import (
    mock_copter_drone,
    mock_state_kwargs,
    mock_shutdown_message,
    mock_message,
)


class TestStatusMessage2(unittest.TestCase):
    class MockState(BaseState):
        pass

    def setUp(self):
        state_args = mock_state_kwargs({
            "outcomes": ["success", "error"],
            "name": "TESTING NAME MockState",
        })
        self.state = TestStatusMessage2.MockState(**state_args)
        self.state.drone = mock_copter_drone()
        self.state.mqtt_client = Mock()
        self.timestamp = "2024-02-08T02:15:55.512+00:00"
        self.state.mqtt_client.timestamp.return_value = self.timestamp


    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    def test_init(
        self,
        mock_RepeatTimer: NonCallableMagicMock
    ):
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion
        )

        mock_armed = True
        mock_battery = Battery(voltage=2.3, current=5.6, level=65.3)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(123.5, Quaternion(1., 0., 0., 1.))
        mock_drone_heading = EnuYaw(1.32)
        mock_geofence_status = False
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(342.53, Quaternion(0., 1., 0., 1.))
        mock_ground_speed = 8.7
        mock_heartbeat_status = HeartbeatStatus.HOVER
        mock_location = PositionDataLla()
        mock_location.add_position(
            23.4,
            LlaPosition(34.2, -102.5, 234.2, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(33.4, -103.7, 12.8, DATUM_REFERENCE.AMSL)
        mock_uav_id = "test drone"

        assert self.state.drone is not None

        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'


        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id
        self.state.drone.gimbal.attitude = mock_gimbal_attitude

        mock_gimbal_heading = self.state.drone.gimbal.heading

        StatusMessage(self.state, 2.3)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.43))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})
        # - after the repeat timer message is processed, check that mqtt.publish is called with the 
        # appropriate inputs - specifically the data dict in the appropriate shape
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        self.assertDictEqual(
            self.state.mqtt_client.publish.call_args[0][1],
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState", # this is the name of the state as given in the mission
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status" : mock_heartbeat_status.name,
                    "gimbal_attitude" : {
                                            "x": mock_gimbal_attitude.get_attitude().x,
                                            "y": mock_gimbal_attitude.get_attitude().y,
                                            "z": mock_gimbal_attitude.get_attitude().z,
                                            "w": mock_gimbal_attitude.get_attitude().w
                                        },
                    "gimbal_heading" : mock_gimbal_heading,
                    "drone_attitude": {
                                        "x": mock_drone_attitude.get_attitude().x,
                                        "y": mock_drone_attitude.get_attitude().y,
                                        "z": mock_drone_attitude.get_attitude().z,
                                        "w": mock_drone_attitude.get_attitude().w
                                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                }
            }
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    def test_optional_fields_in_payload(
        self,
        mock_RepeatTimer: NonCallableMagicMock
    ):
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion,
            GPSStatus,
            Vibration,
            RCOut,
            EstimatorStatusData,
        )

        # Required/base fields
        mock_armed = True
        mock_battery = Battery(voltage=12.5, current=7.8, level=88.0)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(111.0, Quaternion(0.1, 0.2, 0.3, 0.9))
        mock_drone_heading = EnuYaw(2.34)
        mock_geofence_status = True
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(222.22, Quaternion(0.4, 0.5, 0.6, 0.7))
        mock_ground_speed = 12.3
        mock_heartbeat_status = HeartbeatStatus.CONTINUE
        mock_location = PositionDataLla()
        mock_location.add_position(
            55.5,
            LlaPosition(40.1, -75.9, 321.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(41.2, -76.3, 50.0, DATUM_REFERENCE.AMSL)
        mock_uav_id = "uav-optional"

        # Optional fields
        gps_status = GPSStatus(satellites_visible=10, hdop=0.7, vdop=0.9)
        vibration = Vibration(x=0.01, y=0.02, z=0.03, clipping0=0, clipping1=1, clipping2=2)
        rc_out = RCOut(channels=(1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700))
        ekf_status = EstimatorStatusData(
            attitude_status_flag=True,
            velocity_horiz_status_flag=True,
            velocity_vert_status_flag=True,
            pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=True,
            pos_vert_abs_status_flag=True,
            pos_vert_agl_status_flag=True,
            const_pos_mode_status_flag=False,
            pred_pos_horiz_rel_status_flag=False,
            pred_pos_horiz_abs_status_flag=False,
            gps_glitch_status_flag=False,
            accel_error_status_flag=False,
        )
        target_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)

        assert self.state.drone is not None

        # Air lease FSM for air_lease_state
        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'

        # Populate drone data
        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id

        # Optional data population
        self.state.drone.data.gps_status = gps_status
        self.state.drone.data.vibration = vibration
        self.state.drone.data.rc_out = rc_out
        self.state.drone.data.ekf_status = ekf_status
        self.state.drone.data.target_attitude = target_attitude

        # Gimbal
        self.state.drone.gimbal.attitude = mock_gimbal_attitude
        mock_gimbal_heading = self.state.drone.gimbal.heading

        # Init component and request optional fields
        status_component = StatusMessage(self.state, 2.3)
        optional_field_names = [
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
            "target_attitude",
        ]
        status_component.update_status_message(optional_field_names)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.99))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})

        # Validate published payload includes the optional fields
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        self.assertDictEqual(
            self.state.mqtt_client.publish.call_args[0][1],
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState",
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status": mock_heartbeat_status.name,
                    "gimbal_attitude": {
                        "x": mock_gimbal_attitude.get_attitude().x,
                        "y": mock_gimbal_attitude.get_attitude().y,
                        "z": mock_gimbal_attitude.get_attitude().z,
                        "w": mock_gimbal_attitude.get_attitude().w,
                    },
                    "gimbal_heading": mock_gimbal_heading,
                    "drone_attitude": {
                        "x": mock_drone_attitude.get_attitude().x,
                        "y": mock_drone_attitude.get_attitude().y,
                        "z": mock_drone_attitude.get_attitude().z,
                        "w": mock_drone_attitude.get_attitude().w,
                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                    # Optional fields should be JSON-serializable dicts
                    "gps_status": gps_status.to_dict(),
                    "vibration": vibration.to_dict(),
                    "rc_out": rc_out.to_dict(),
                    "ekf_status": ekf_status.to_dict(),
                    "target_attitude": {
                        "x": target_attitude.x,
                        "y": target_attitude.y,
                        "z": target_attitude.z,
                        "w": target_attitude.w,
                    },
                },
            },
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    @patch.object(StatusMessage_module, "rospy")
    def test_optional_fields_via_mqtt_no_ekf_status(
        self,
        mock_rospy: NonCallableMagicMock,
        mock_RepeatTimer: NonCallableMagicMock,
    ):
        import json as _json
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion,
            GPSStatus,
            Vibration,
            RCOut,
        )
        from .mock_types import mock_mqtt_message

        mock_rospy.loginfo.side_effect = print
        mock_rospy.logwarn.side_effect = print

        print("#" * 100)

        # Required/base fields
        mock_armed = True
        mock_battery = Battery(voltage=11.1, current=6.2, level=77.0)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(101.0, Quaternion(0.2, 0.1, 0.0, 0.97))
        mock_drone_heading = EnuYaw(1.11)
        mock_geofence_status = False
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(333.33, Quaternion(0.6, 0.3, 0.2, 0.7))
        mock_ground_speed = 6.7
        mock_heartbeat_status = HeartbeatStatus.HOVER
        mock_location = PositionDataLla()
        mock_location.add_position(
            12.34,
            LlaPosition(35.0, -106.6, 200.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(36.1, -107.1, 80.0, DATUM_REFERENCE.AMSL)
        mock_uav_id = "uav-mqtt-optional"

        # Optional fields (exclude ekf_status on purpose)
        gps_status = GPSStatus(satellites_visible=7, hdop=0.8, vdop=1.1)
        vibration = Vibration(x=0.05, y=0.06, z=0.07, clipping0=0, clipping1=0, clipping2=1)
        rc_out = RCOut(channels=(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        target_attitude = Quaternion(0.0, 0.0, 0.7071, 0.7071)

        assert self.state.drone is not None

        # Air lease FSM for air_lease_state
        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'

        # Populate drone data
        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id

        # Optional data population (leave ekf_status as None)
        self.state.drone.data.gps_status = gps_status
        self.state.drone.data.vibration = vibration
        self.state.drone.data.rc_out = rc_out
        self.state.drone.data.target_attitude = target_attitude

        # Gimbal
        self.state.drone.gimbal.attitude = mock_gimbal_attitude
        mock_gimbal_heading = self.state.drone.gimbal.heading

        # Init component
        _ = StatusMessage(self.state, 2.3)

        # Create and queue MQTT command to update optional fields
        requested_fields = [
            "gps_status",
            "vibration",
            "rc_out",
            "target_attitude",
        ]
        payload = _json.dumps({"fields": requested_fields})
        topic = f"drone/{self.state.uav_id}/update_status_message"
        update_status_msg = mock_mqtt_message("update_status_message", topic, payload)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        # Put the update command first, so fields are set before the timer send
        self.state.message_queue.put(update_status_msg)
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.77))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})

        # take a look at state.status_message.optional_fields to make sure it matches our list
        self.assertEqual(
            self.state.status_message.optional_fields,
            requested_fields
        )

        # Validate published payload includes the requested optional fields
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        actual = self.state.mqtt_client.publish.call_args[0][1]
        self.assertDictEqual(
            actual,
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState",
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status": mock_heartbeat_status.name,
                    "gimbal_attitude": {
                        "x": mock_gimbal_attitude.get_attitude().x,
                        "y": mock_gimbal_attitude.get_attitude().y,
                        "z": mock_gimbal_attitude.get_attitude().z,
                        "w": mock_gimbal_attitude.get_attitude().w,
                    },
                    "gimbal_heading": mock_gimbal_heading,
                    "drone_attitude": {
                        "x": mock_drone_attitude.get_attitude().x,
                        "y": mock_drone_attitude.get_attitude().y,
                        "z": mock_drone_attitude.get_attitude().z,
                        "w": mock_drone_attitude.get_attitude().w,
                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                    # Optional fields (no ekf_status requested)
                    "gps_status": gps_status.to_dict(),
                    "vibration": vibration.to_dict(),
                    "rc_out": rc_out.to_dict(),
                    "target_attitude": {
                        "x": target_attitude.x,
                        "y": target_attitude.y,
                        "z": target_attitude.z,
                        "w": target_attitude.w,
                    },
                },
            },
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    def test_optional_fields_missing_ekf_status(
        self,
        mock_RepeatTimer: NonCallableMagicMock
    ):
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion,
            GPSStatus,
            Vibration,
            RCOut,
        )

        # Required/base fields
        mock_armed = True
        mock_battery = Battery(voltage=11.1, current=6.2, level=77.0)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(101.0, Quaternion(0.2, 0.1, 0.0, 0.97))
        mock_drone_heading = EnuYaw(1.11)
        mock_geofence_status = False
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(333.33, Quaternion(0.6, 0.3, 0.2, 0.7))
        mock_ground_speed = 6.7
        mock_heartbeat_status = HeartbeatStatus.HOVER
        mock_location = PositionDataLla()
        mock_location.add_position(
            12.34,
            LlaPosition(35.0, -106.6, 200.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(36.1, -107.1, 80.0, DATUM_REFERENCE.AMSL)
        mock_uav_id = "uav-missing-ekf"

        # Optional fields (exclude ekf_status on purpose)
        gps_status = GPSStatus(satellites_visible=7, hdop=0.8, vdop=1.1)
        vibration = Vibration(x=0.05, y=0.06, z=0.07, clipping0=0, clipping1=0, clipping2=1)
        rc_out = RCOut(channels=(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        target_attitude = Quaternion(0.0, 0.0, 0.7071, 0.7071)

        assert self.state.drone is not None

        # Air lease FSM for air_lease_state
        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'

        # Populate drone data
        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id

        # Optional data population (leave ekf_status as None)
        self.state.drone.data.gps_status = gps_status
        self.state.drone.data.vibration = vibration
        self.state.drone.data.rc_out = rc_out
        self.state.drone.data.target_attitude = target_attitude

        # Gimbal
        self.state.drone.gimbal.attitude = mock_gimbal_attitude
        mock_gimbal_heading = self.state.drone.gimbal.heading

        # Init component and request ALL optional fields (including ekf_status)
        status_component = StatusMessage(self.state, 2.3)
        optional_field_names = [
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
            "target_attitude",
        ]
        status_component.update_status_message(optional_field_names)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.77))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})

        # Validate published payload includes all requested optional fields,
        # with ekf_status present but as None
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        self.assertDictEqual(
            self.state.mqtt_client.publish.call_args[0][1],
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState",
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status": mock_heartbeat_status.name,
                    "gimbal_attitude": {
                        "x": mock_gimbal_attitude.get_attitude().x,
                        "y": mock_gimbal_attitude.get_attitude().y,
                        "z": mock_gimbal_attitude.get_attitude().z,
                        "w": mock_gimbal_attitude.get_attitude().w,
                    },
                    "gimbal_heading": mock_gimbal_heading,
                    "drone_attitude": {
                        "x": mock_drone_attitude.get_attitude().x,
                        "y": mock_drone_attitude.get_attitude().y,
                        "z": mock_drone_attitude.get_attitude().z,
                        "w": mock_drone_attitude.get_attitude().w,
                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                    # Optional fields, ekf_status should be None
                    "gps_status": gps_status.to_dict(),
                    "vibration": vibration.to_dict(),
                    "rc_out": rc_out.to_dict(),
                    "ekf_status": None,
                    "target_attitude": {
                        "x": target_attitude.x,
                        "y": target_attitude.y,
                        "z": target_attitude.z,
                        "w": target_attitude.w,
                    },
                },
            },
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    def test_optional_fields_no_ekf_status_requested(
        self,
        mock_RepeatTimer: NonCallableMagicMock
    ):
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion,
            GPSStatus,
            Vibration,
            RCOut,
        )

        # Required/base fields
        mock_armed = True
        mock_battery = Battery(voltage=11.1, current=6.2, level=77.0)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(101.0, Quaternion(0.2, 0.1, 0.0, 0.97))
        mock_drone_heading = EnuYaw(1.11)
        mock_geofence_status = False
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(333.33, Quaternion(0.6, 0.3, 0.2, 0.7))
        mock_ground_speed = 6.7
        mock_heartbeat_status = HeartbeatStatus.HOVER
        mock_location = PositionDataLla()
        mock_location.add_position(
            12.34,
            LlaPosition(35.0, -106.6, 200.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(36.1, -107.1, 80.0, DATUM_REFERENCE.AMSL)
        mock_uav_id = "uav-missing-ekf"

        # Optional fields (exclude ekf_status on purpose)
        gps_status = GPSStatus(satellites_visible=7, hdop=0.8, vdop=1.1)
        vibration = Vibration(x=0.05, y=0.06, z=0.07, clipping0=0, clipping1=0, clipping2=1)
        rc_out = RCOut(channels=(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        target_attitude = Quaternion(0.0, 0.0, 0.7071, 0.7071)

        assert self.state.drone is not None

        # Air lease FSM for air_lease_state
        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'

        # Populate drone data
        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id

        # Optional data population (leave ekf_status as None)
        self.state.drone.data.gps_status = gps_status
        self.state.drone.data.vibration = vibration
        self.state.drone.data.rc_out = rc_out
        self.state.drone.data.target_attitude = target_attitude

        # Gimbal
        self.state.drone.gimbal.attitude = mock_gimbal_attitude
        mock_gimbal_heading = self.state.drone.gimbal.heading

        # Init component and request ALL optional fields (including ekf_status)
        status_component = StatusMessage(self.state, 2.3)
        optional_field_names = [
            "gps_status",
            "vibration",
            "rc_out",
            "target_attitude",
            # "ekf_status" is excluded intentionally
        ]
        status_component.update_status_message(optional_field_names)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.77))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})

        # Validate published payload includes all requested optional fields,
        # with ekf_status present but as None
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        self.assertDictEqual(
            self.state.mqtt_client.publish.call_args[0][1],
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState",
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status": mock_heartbeat_status.name,
                    "gimbal_attitude": {
                        "x": mock_gimbal_attitude.get_attitude().x,
                        "y": mock_gimbal_attitude.get_attitude().y,
                        "z": mock_gimbal_attitude.get_attitude().z,
                        "w": mock_gimbal_attitude.get_attitude().w,
                    },
                    "gimbal_heading": mock_gimbal_heading,
                    "drone_attitude": {
                        "x": mock_drone_attitude.get_attitude().x,
                        "y": mock_drone_attitude.get_attitude().y,
                        "z": mock_drone_attitude.get_attitude().z,
                        "w": mock_drone_attitude.get_attitude().w,
                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                    # Optional fields, ekf_status should be None
                    "gps_status": gps_status.to_dict(),
                    "vibration": vibration.to_dict(),
                    "rc_out": rc_out.to_dict(),
                    "target_attitude": target_attitude.to_dict(),
                },
            },
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.components.StatusMessage"], "RepeatTimer")
    def test_serialize_optional_fields_in_payload(
        self,
        mock_RepeatTimer: NonCallableMagicMock
    ):
        from dr_onboard_autonomy.states.components import StatusMessage
        from dr_onboard_autonomy.models import (
            DATUM_REFERENCE,
            AttitudeData,
            Battery,
            EnuYaw,
            FCUCopterMode,
            FCUStatus,
            HeartbeatStatus,
            LlaPosition,
            PositionDataLla,
            Quaternion,
            GPSStatus,
            Vibration,
            RCOut,
            EstimatorStatusData,
        )

        # Required/base fields
        mock_armed = True
        mock_battery = Battery(voltage=12.5, current=7.8, level=88.0)
        mock_drone_attitude = AttitudeData()
        mock_drone_attitude.add_attitude(111.0, Quaternion(0.1, 0.2, 0.3, 0.9))
        mock_drone_heading = EnuYaw(2.34)
        mock_geofence_status = True
        mock_gimbal_attitude = AttitudeData()
        mock_gimbal_attitude.add_attitude(222.22, Quaternion(0.4, 0.5, 0.6, 0.7))
        mock_ground_speed = 12.3
        mock_heartbeat_status = HeartbeatStatus.CONTINUE
        mock_location = PositionDataLla()
        mock_location.add_position(
            55.5,
            LlaPosition(40.1, -75.9, 321.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        )
        mock_mode = FCUCopterMode.OFFBOARD
        mock_status = FCUStatus.ACTIVE
        mock_target_position = LlaPosition(41.2, -76.3, 50.0, DATUM_REFERENCE.AMSL)
        mock_uav_id = "uav-optional"

        # Optional fields
        gps_status = GPSStatus(satellites_visible=10, hdop=0.7, vdop=0.9)
        vibration = Vibration(x=0.01, y=0.02, z=0.03, clipping0=0, clipping1=1, clipping2=2)
        rc_out = RCOut(channels=(1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700))
        ekf_status = EstimatorStatusData(
            attitude_status_flag=True,
            velocity_horiz_status_flag=True,
            velocity_vert_status_flag=True,
            pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=True,
            pos_vert_abs_status_flag=True,
            pos_vert_agl_status_flag=True,
            const_pos_mode_status_flag=False,
            pred_pos_horiz_rel_status_flag=False,
            pred_pos_horiz_abs_status_flag=False,
            gps_glitch_status_flag=False,
            accel_error_status_flag=False,
        )
        target_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)

        assert self.state.drone is not None

        # Air lease FSM for air_lease_state
        model, machine = make_model_and_machine(self.state.uav_id, self.state.mqtt_client)
        self.state.air_lease_protocol_fsm = model
        assert self.state.air_lease_protocol_fsm is not None
        assert self.state.air_lease_protocol_fsm.state.name == 'IDLE'

        # Populate drone data
        self.state.drone.data.armed = mock_armed
        self.state.drone.data.battery = mock_battery
        self.state.drone.data.attitude = mock_drone_attitude
        self.state.drone.data.heading = mock_drone_heading
        self.state.drone.data.geofence_status = mock_geofence_status
        self.state.drone.data.ground_speed = mock_ground_speed
        self.state.drone.data.heartbeat_status = mock_heartbeat_status
        self.state.drone.data.location = mock_location
        self.state.drone.data.mode = mock_mode
        self.state.drone.data.status = mock_status
        self.state.drone.data.target_position = mock_target_position
        self.state.drone.data.uav_id = mock_uav_id

        # Optional data population
        self.state.drone.data.gps_status = gps_status
        self.state.drone.data.vibration = vibration
        self.state.drone.data.rc_out = rc_out
        self.state.drone.data.ekf_status = ekf_status
        self.state.drone.data.target_attitude = target_attitude

        # Gimbal
        self.state.drone.gimbal.attitude = mock_gimbal_attitude
        mock_gimbal_heading = self.state.drone.gimbal.heading

        # Init component and request optional fields
        status_component = StatusMessage(self.state, 2.3)
        optional_field_names = [
            "target_attitude",
            "gps_status",
            "vibration",
            "rc_out",
            "ekf_status",
        ]
        status_component.update_status_message(optional_field_names)

        TIMER_NAME = "StatusMessage__send_status_timer"
        mock_RepeatTimer.assert_called_once_with(TIMER_NAME, 2.3)
        self.assertIn(mock_RepeatTimer.return_value, self.state.message_senders)

        self.state.message_queue = Queue()
        self.state.message_queue.put(mock_message(TIMER_NAME, 0.99))
        self.state.message_queue.put(mock_shutdown_message())

        self.state.execute({})

        # Validate published payload includes the optional fields
        self.assertEqual(self.state.mqtt_client.publish.call_args[0][0], "update_drone")
        self.maxDiff = None
        actual = self.state.mqtt_client.publish.call_args[0][1]
        self.assertDictEqual(
            actual,
            {
                "uavid": mock_uav_id,
                "timestamp": self.timestamp,
                "status": {
                    "status": mock_status.name,
                    "mode": mock_mode.name,
                    "onboard_pilot": "TESTING NAME MockState",
                    "state_type": "MockState",
                    "speed": mock_ground_speed,
                    "location": {
                        "latitude": mock_location.get_position().latitude,
                        "longitude": mock_location.get_position().longitude,
                        "altitude": mock_location.get_position().to_amsl().altitude,
                    },
                    "armed": mock_armed,
                    "battery": {
                        "voltage": mock_battery.voltage,
                        "current": mock_battery.current,
                        "level": mock_battery.level,
                    },
                    "geofence": mock_geofence_status,
                    "heartbeat_status": mock_heartbeat_status.name,
                    "gimbal_attitude": {
                        "x": mock_gimbal_attitude.get_attitude().x,
                        "y": mock_gimbal_attitude.get_attitude().y,
                        "z": mock_gimbal_attitude.get_attitude().z,
                        "w": mock_gimbal_attitude.get_attitude().w,
                    },
                    "gimbal_heading": mock_gimbal_heading,
                    "drone_attitude": {
                        "x": mock_drone_attitude.get_attitude().x,
                        "y": mock_drone_attitude.get_attitude().y,
                        "z": mock_drone_attitude.get_attitude().z,
                        "w": mock_drone_attitude.get_attitude().w,
                    },
                    "drone_heading": mock_drone_heading,
                    "air_lease_state": 'IDLE',
                    # Optional fields should be JSON-serializable dicts
                    "gps_status": gps_status.to_dict(),
                    "vibration": vibration.to_dict(),
                    "rc_out": rc_out.to_dict(),
                    "ekf_status": ekf_status.to_dict(),
                    "target_attitude": {
                        "x": target_attitude.x,
                        "y": target_attitude.y,
                        "z": target_attitude.z,
                        "w": target_attitude.w,
                    },
                },
            },
        )
        result = json.dumps(actual, indent=4)  # For pretty printing in test output
        # THIS needs to work to pass the test
        self.assertIsInstance(result, str)

