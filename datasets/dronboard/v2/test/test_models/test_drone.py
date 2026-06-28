import unittest
from unittest.mock import Mock

from dr_onboard_autonomy.models import (
    ALTITUDE_REFERENCE,
    CopterDrone,
    EnuYaw,
    FCUCopterMode,
    LlaPosition,
    NedPosition,
    NedVelocity
)


class TestAbstractCopterDrone(unittest.TestCase):

    def setUp(self):
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


        self.test_copter_drone = TestCopterDrone(None, "test_drone")


    def test_init(self):
        self.test_copter_drone.uav_name = "test_drone"


    def test_set_get_fcu_mode(self):
        # check fcu mode is None until set
        self.assertIsNone(self.test_copter_drone.get_fcu_mode())

        self.test_copter_drone._set_fcu_mode.return_value = True

        self.assertTrue(self.test_copter_drone.set_fcu_mode(FCUCopterMode.POS_HOLD))
        self.test_copter_drone._set_fcu_mode.assert_called_once_with(FCUCopterMode.POS_HOLD)

        self.assertEqual(self.test_copter_drone.get_fcu_mode(), FCUCopterMode.POS_HOLD)


    def test_fcu_mode_set_failure(self):
        self.test_copter_drone._set_fcu_mode.return_value = False

        self.assertFalse(self.test_copter_drone.set_fcu_mode(FCUCopterMode.POS_HOLD))

        # mode should still be the None default because not successfully set
        self.assertIsNone(self.test_copter_drone.get_fcu_mode())


    def test_send_lla_setpoint(self):
        lla_setpoint = LlaPosition(
            latitude=37.36,
            longitude=-103.23,
            altitude=500.21,
            altitude_ref=ALTITUDE_REFERENCE.ELLIPSOID_WGS84
        )
        yaw_setpoint = EnuYaw(2.56)

        self.test_copter_drone.send_lla_setpoint(
            lla=lla_setpoint,
            yaw=yaw_setpoint
        )

        self.assertEqual(self.test_copter_drone.data.setpoint_latest.lla, lla_setpoint)
        self.assertEqual(self.test_copter_drone.data.setpoint_latest.yaw, yaw_setpoint)
        self.test_copter_drone._send_lla_setpoint.assert_called_once_with(
            lla=lla_setpoint,
            yaw=yaw_setpoint
        )


    def test_send_ned_position_setpoint(self):
        ned_setpoint = NedPosition(
            north=10.0,
            east=0.0,
            down=5.2
        )
        yaw_setpoint = EnuYaw(1.23)

        self.test_copter_drone.send_ned_setpoint(
            ned_position=ned_setpoint,
            yaw=yaw_setpoint
        )

        self.assertEqual(self.test_copter_drone.data.setpoint_latest.ned_position, ned_setpoint)
        self.assertEqual(self.test_copter_drone.data.setpoint_latest.yaw, yaw_setpoint)
        self.test_copter_drone._send_ned_setpoint.assert_called_once_with(
            ned_position=ned_setpoint,
            yaw=yaw_setpoint
        )


    def test_send_ned_velocity_setpoint(self):
        ned_velocity_setpoint = NedVelocity(
            north=3.0,
            east=1.0,
            down=0.2
        )
        yaw_setpoint = EnuYaw(0.85)

        self.test_copter_drone.send_ned_velocity_setpoint(
            ned_velocity=ned_velocity_setpoint,
            yaw=yaw_setpoint
        )

        self.assertEqual(
            self.test_copter_drone.data.setpoint_latest.ned_velocity,
            ned_velocity_setpoint
        )
        self.assertEqual(self.test_copter_drone.data.setpoint_latest.yaw, yaw_setpoint)
        self.test_copter_drone._send_ned_velocity_setpoint.assert_called_once_with(
            ned_velocity=ned_velocity_setpoint,
            yaw=yaw_setpoint
        )
