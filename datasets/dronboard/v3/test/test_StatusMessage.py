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
            "name": "MockState",
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
        from dr_onboard_autonomy.states.components import StatusMessage2
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

        StatusMessage2(self.state, 2.3)

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
                    "onboard_pilot": "MockState",
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
                    "drone_heading": mock_drone_heading
                }
            }
        )

