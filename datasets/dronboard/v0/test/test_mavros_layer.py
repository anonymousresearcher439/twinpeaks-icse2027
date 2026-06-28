#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_mavros_layer"
import roslib
import rospy
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget

roslib.load_manifest(PKG)

import unittest
from unittest.mock import Mock, NonCallableMock, patch

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.states import Arm, Disarm, Preflight

now_func = Mock()
now_func.return_value = 0.0
@patch('dr_onboard_autonomy.mavros_layer.rospy.Time.now')
class TestMavrosLayer(unittest.TestCase):
    def build_mock_publisher(self):
        return NonCallableMock(pub=Mock(return_value=None))

    def build_mock_drone(self):
        attr = {
            "_build_local_setpoint.side_effect": MAVROSDrone._build_local_setpoint,
            "_build_global_setpoint.side_effect": MAVROSDrone._build_global_setpoint,
        }
        return NonCallableMock(
            spec=MAVROSDrone,
            _setpoint_global_pub=self.build_mock_publisher(),
            _setpoint_local_pub=self.build_mock_publisher(),
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


class MavrosLayerTestSuite(TestMavrosLayer):
    pass


if __name__ == "__main__":
    import rostest

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, MavrosLayerTestSuite)
