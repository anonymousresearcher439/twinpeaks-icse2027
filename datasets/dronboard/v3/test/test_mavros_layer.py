#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_mavros_layer"
import roslib
import rospy
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget

roslib.load_manifest(PKG)

import math
import unittest
from unittest.mock import Mock, NonCallableMock, NonCallableMagicMock, patch
from threading import Lock


from dr_onboard_autonomy.drone_data import Data
from dr_onboard_autonomy.mavros_layer import MAVROSDrone, SetpointType

from .mock_types import mock_drone

now_func = Mock()
now_func.return_value = 0.0
@patch('dr_onboard_autonomy.mavros_layer.rospy.Time.now')
class TestMavrosLayer(unittest.TestCase):
    def build_mock_publisher(self):
        return NonCallableMock(pub=Mock(return_value=None))

    def build_mock_thread_lock(self):
        mock_lock = NonCallableMagicMock()
        mock_lock.__enter__.return_value = Mock()

        return mock_lock

    def build_mock_drone(self):
        attr = {
            "_build_local_setpoint.side_effect": MAVROSDrone._build_local_setpoint,
            "_build_global_setpoint.side_effect": MAVROSDrone._build_global_setpoint,
            "update_data.side_effect": MAVROSDrone.update_data,
            "get_data.side_effect": MAVROSDrone.get_data
        }
        return NonCallableMock(
            spec=MAVROSDrone,
            _setpoint_global_pub=self.build_mock_publisher(),
            _setpoint_local_pub=self.build_mock_publisher(),
            _data=Data(),
            _lock=self.build_mock_thread_lock(),
            **attr,
        )

    def test_send_setpoint_too_many_args(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(
            mock_drone, lla=(1, 2, 3), ned_position=(4, 5, 6)
        )
        self.assertFalse(result)
        result = MAVROSDrone.send_setpoint(
            mock_drone, lla=(1, 2, 3), ned_velocity=(4, 5, 6)
        )
        self.assertFalse(result)
        result = MAVROSDrone.send_setpoint(
            mock_drone, ned_position=(1, 2, 3), ned_velocity=(4, 5, 6)
        )
        self.assertFalse(result)
        result = MAVROSDrone.send_setpoint(
            mock_drone, lla=(1, 2, 3), ned_position=(4, 5, 6), ned_velocity=(7, 8, 9)
        )
        self.assertFalse(result)

    def test_send_setpoint_bad_tuple_args(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(mock_drone, lla=(1, 2))
        self.assertFalse(result)
        result = MAVROSDrone.send_setpoint(mock_drone, ned_position=(3, 4))
        self.assertFalse(result)
        result = MAVROSDrone.send_setpoint(mock_drone, ned_velocity=(5, 6))
        self.assertFalse(result)

    @unittest.skip("switched to /mavros/setpoint_position/global messages, this test is out of date")
    def test_send_setpoint_with_lla_no_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(mock_drone, lla=(1, 2, 3))

        pub = mock_drone._setpoint_global_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), GlobalPositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 0)
        self.assertEqual(setpoint.latitude, 1.0)
        self.assertEqual(setpoint.longitude, 2.0)
        self.assertEqual(setpoint.altitude, 3.0)
        self.assertEqual(setpoint.type_mask, 0b111_111_111_000)

    def test_send_setpoint_with_ned_pos2(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(mock_drone, ned_position=(10.0, 0.0, -10.0))

        pub = mock_drone._setpoint_local_pub
        self.assertTrue(pub.publish.called)
        (local_pos,) = pub.publish.call_args.args

        self.assertEqual(type(local_pos), PositionTarget)
        self.assertEqual(local_pos.coordinate_frame, 1)
        self.assertEqual(local_pos.position.x, 10.0)
        self.assertEqual(local_pos.position.y, 0.0)
        self.assertEqual(local_pos.position.z, -10.0)
        self.assertEqual(local_pos.type_mask, 0b111_111_111_000)

    @unittest.skip("switched to /mavros/setpoint_position/global messages, this test is out of date")
    def test_send_setpoint_with_lla_with_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(
            mock_drone, lla=(1, 2, 3), yaw=4.0, is_yaw_set=True
        )

        pub = mock_drone._setpoint_global_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), GlobalPositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 0)
        self.assertEqual(setpoint.latitude, 1.0)
        self.assertEqual(setpoint.longitude, 2.0)
        self.assertEqual(setpoint.altitude, 3.0)
        self.assertEqual(setpoint.yaw, 4.0)
        self.assertEqual(setpoint.type_mask, 0b101_111_111_000)

    def test_send_setpoint_with_ned_pos_no_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(mock_drone, ned_position=(1, 2, 3))

        pub = mock_drone._setpoint_local_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), PositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 1)
        self.assertEqual(setpoint.position.x, 1.0)
        self.assertEqual(setpoint.position.y, 2.0)
        self.assertEqual(setpoint.position.z, 3.0)
        self.assertEqual(setpoint.type_mask, 0b111_111_111_000)

    def test_send_setpoint_with_ned_pos_with_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(
            mock_drone, ned_position=(1, 2, 3), yaw=4.0, is_yaw_set=True
        )

        pub = mock_drone._setpoint_local_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), PositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 1)
        self.assertEqual(setpoint.position.x, 1.0)
        self.assertEqual(setpoint.position.y, 2.0)
        self.assertEqual(setpoint.position.z, 3.0)
        self.assertEqual(setpoint.yaw, 4.0)
        self.assertEqual(setpoint.type_mask, 0b101_111_111_000)

    def test_send_setpoint_with_ned_vel_no_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(mock_drone, ned_velocity=(1, 2, 3))

        pub = mock_drone._setpoint_local_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), PositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 1)
        self.assertEqual(setpoint.velocity.x, 1.0)
        self.assertEqual(setpoint.velocity.y, 2.0)
        self.assertEqual(setpoint.velocity.z, 3.0)
        self.assertEqual(setpoint.type_mask, 0b111_111_000_111)

    def test_send_setpoint_with_ned_vel_with_yaw(self, time_func):
        mock_drone = self.build_mock_drone()
        result = MAVROSDrone.send_setpoint(
            mock_drone, ned_velocity=(1, 2, 3), yaw=4.0, is_yaw_set=True
        )

        pub = mock_drone._setpoint_local_pub
        self.assertTrue(pub.publish.called)
        (setpoint,) = pub.publish.call_args.args

        self.assertEqual(type(setpoint), PositionTarget)
        self.assertEqual(setpoint.coordinate_frame, 1)
        self.assertEqual(setpoint.velocity.x, 1.0)
        self.assertEqual(setpoint.velocity.y, 2.0)
        self.assertEqual(setpoint.velocity.z, 3.0)
        self.assertEqual(setpoint.yaw, 4.0)
        self.assertEqual(setpoint.type_mask, 0b101_111_000_111)

    def test_build_local_position_setpoint(self, time_func):
        local_pos = MAVROSDrone._build_local_setpoint(1.0, 2.0, 3.0, is_velocity=False)
        self.assertEqual(type(local_pos), PositionTarget)
        self.assertEqual(local_pos.coordinate_frame, 1)
        self.assertEqual(local_pos.position.x, 1.0)
        self.assertEqual(local_pos.position.y, 2.0)
        self.assertEqual(local_pos.position.z, 3.0)
        self.assertEqual(local_pos.type_mask, 0b111_111_111_000)

    def test_build_local_velocity_setpoint(self, time_func):
        local_vel = MAVROSDrone._build_local_setpoint(1.0, 2.0, 3.0)
        self.assertEqual(type(local_vel), PositionTarget)
        self.assertEqual(local_vel.coordinate_frame, 1)
        self.assertEqual(local_vel.velocity.x, 1.0)
        self.assertEqual(local_vel.velocity.y, 2.0)
        self.assertEqual(local_vel.velocity.z, 3.0)
        self.assertEqual(local_vel.type_mask, 0b111_111_000_111)

    @unittest.skip("we don't use /mavros/setpoint_raw/global messages so we don't need _build_global_setpoint anymore. We now use /mavros/setpoint_position/global message and the _build_global_setpoint2 method")
    def test_build_global_setpoint(self, time_func):
        global_pos = MAVROSDrone._build_global_setpoint(1.0, 2.0, 3.0)
        self.assertEqual(type(global_pos), GlobalPositionTarget)
        self.assertEqual(global_pos.coordinate_frame, 0)
        self.assertEqual(global_pos.latitude, 1.0)
        self.assertEqual(global_pos.longitude, 2.0)
        self.assertEqual(global_pos.altitude, 3.0)
        self.assertEqual(global_pos.type_mask, 0b111_111_111_000)
    
    def test_setpoint_property(self, time_func):

        import dr_onboard_autonomy.mavros_layer as mavros_layer

        with patch(target='dr_onboard_autonomy.mavros_layer.rospy'):
            drone = MAVROSDrone('test_uav_name', False, False)
        setpoint_type, vec3, yaw = drone.setpoint
        self.assertEqual(setpoint_type, None)
        self.assertEqual(vec3, None)
        self.assertEqual(yaw, None)

        lla_pos = (41.60667737424942, -86.35540182897999, 240.0)
        drone.send_setpoint(lla=lla_pos, yaw=3.14, is_yaw_set=True)
        setpoint_type, vec3, yaw = drone.setpoint
        self.assertEqual(setpoint_type, SetpointType.LLA)
        self.assertEqual(vec3, lla_pos)
        self.assertEqual(yaw, 3.14)

        ned_pos = (0.0, 10.0, 20.0)
        yaw_setpoint = math.pi / 2
        drone.send_setpoint(ned_position=ned_pos, yaw=yaw_setpoint, is_yaw_set=True)
        setpoint_type, vec3, yaw = drone.setpoint
        self.assertEqual(setpoint_type, SetpointType.NED_POSITION)
        self.assertEqual(vec3, ned_pos)
        self.assertEqual(yaw, yaw_setpoint)

        ned_vel = (0.0, 10.0, 20.0)
        yaw_setpoint2 = math.pi / 4
        drone.send_setpoint(ned_velocity=ned_vel, yaw=yaw_setpoint2, is_yaw_set=True)
        setpoint_type, vec3, yaw = drone.setpoint
        self.assertEqual(setpoint_type, SetpointType.NED_VELOCITY)
        self.assertEqual(vec3, ned_vel)
        self.assertEqual(yaw, yaw_setpoint2)

    def test_drone_data(self, time_func):
        mock_drone = self.build_mock_drone()
        mock_drone.update_data(mock_drone, "gimbal_attitude", {
            "x": 2.2513,
            "y": 0.0,
            "z": 0.0,
            "w": 0.9871
        })

        self.assertEqual(mock_drone._data.gimbal_attitude, {
            "x": 2.2513,
            "y": 0.0,
            "z": 0.0,
            "w": 0.9871
        })
        self.assertEqual(mock_drone._data.to_dict()["status"]["gimbal_attitude"], {
            "x": 2.2513,
            "y": 0.0,
            "z": 0.0,
            "w": 0.9871
        })
        mock_drone._lock.__enter__.assert_called_once_with()

        gimbal_attitude = mock_drone.get_data(mock_drone, "gimbal_attitude")
        self.assertEqual(gimbal_attitude, {
            "x": 2.2513,
            "y": 0.0,
            "z": 0.0,
            "w": 0.9871
        })


if __name__ == "__main__":
    import rostest

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, MavrosLayerTestSuite)
