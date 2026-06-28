import unittest

from dr_onboard_autonomy.models import (
    Battery,
    FCUState,
    FCUStatus,
    FCUCopterMode,
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
    RelativeAltitude,
    EnuYaw,
    FCU_MODES_HUMAN_CONTROL,
)
from dr_onboard_autonomy.message_converters import (
    ros_battery_adapter,
    ros_attitude_adapter,
    ros_imu_adapter,
    ros_compass_heading_adapter,
    ros_ground_speed_adapter,
    ros_velocity_adapter,
    ros_position_adapter,
    ros_relative_altitude_adapter,
    ros_fcu_status_adapter,
    ros_state_adapter,
)

import mock_types


class MessageConvertersTestCase(unittest.TestCase):
    def test_battery(self):
        voltage = 14
        current = 4
        level = 0.5
        ros_battery = mock_types.mock_ros_battery_data(voltage, current, level)
        actual: Battery = ros_battery_adapter(ros_battery)
        
        self.assertIsInstance(actual, Battery)
        self.assertEqual(actual.voltage, voltage)
        self.assertEqual(actual.current, current)
        self.assertEqual(actual.level, level)
    
    def test_state_message(self):
        ros_state = mock_types.mock_state_message(
            connected=True,
            armed=True,
            guided=True,
            manual_input=False,
            mode="OFFBOARD",
            system_status=3,
            msg_time=None
        )['data']
        actual: FCUState = ros_state_adapter(ros_state)
        
        self.assertIsInstance(actual, FCUState)
        self.assertEqual(actual.armed, True)
        self.assertEqual(actual.connected, True)
        self.assertEqual(actual.mode, FCUCopterMode.OFFBOARD)
        self.assertEqual(actual.status, FCUStatus.STANDBY)
        
    def test_fcu_mode_mapping_for_failsafe(self):
        human_modes = {
            # ardupilot human control modes
            "ACRO",
            "ALT_HOLD",
            "AUTOTUNE",
            "AVOID_ADSB", 
            "BRAKE", 
            "DRIFT",
            "FLIP",
            "GUIDED_NOGPS",
            "POSHOLD",
            "POSITION",
            "RTL",
            "SPORT",
            "STABILIZE",
            "THROW", 
            # px4 human control modes
            "ACRO",
            "ALTCTL", 
            "AUTO.RTGS",
            "AUTO.RTL",
            "MANUAL",
            "POSCTL",
            "RATTITUDE",
            "STABILIZED",
        }

        for mode in human_modes:
            mock_state = mock_types.mock_state_message(
                connected=True,
                armed=True,
                guided=True,
                manual_input=False,
                mode=mode,
                system_status=3,
                msg_time=None
            )['data']
            actual: FCUState = ros_state_adapter(mock_state)
            self.assertIsInstance(actual, FCUState)
            self.assertIn(actual.mode.value, FCU_MODES_HUMAN_CONTROL, f"Mode {mode} not in human control modes. FCUCopterMode: {actual.mode}")
    
    def test_fcu_non_human_modes(self):
        auto_modes = {
            #autopilot modes
            "AUTO",
            #"GUIDED_NOGPS", # TODO should this map to OFFBOARD?
            "GUIDED",
            "LOITER",
            "OF_LOITER",
            # px4 modes
            "AUTO.LOITER",
            "AUTO.MISSION",
            "AUTO.TAKEOFF",
            "OFFBOARD",
        }

        for mode in auto_modes:
            mock_state = mock_types.mock_state_message(
                connected=True,
                armed=True,
                guided=True,
                manual_input=False,
                mode=mode,
                system_status=3,
                msg_time=None
            )['data']
            actual: FCUState = ros_state_adapter(mock_state)
            self.assertIsInstance(actual, FCUState)
            self.assertNotIn(actual.mode.value, FCU_MODES_HUMAN_CONTROL, f"Mode {mode} not in human control modes. FCUCopterMode: {actual.mode}")

    def test_imu(self):
        import math
        def angle_axis_to_quaternion(theta, axis):
            axis_magnitude = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
            # Normalize the axis to make it a unit vector
            x, y, z = axis[0] / axis_magnitude, axis[1] / axis_magnitude, axis[2] / axis_magnitude
            # Calculate the quaternion components
            qw = math.cos(theta / 2.0)
            qx = x * math.sin(theta / 2.0)
            qy = y * math.sin(theta / 2.0)
            qz = z * math.sin(theta / 2.0)
            return (qw, qx, qy, qz)
        
        w, x, y, z = angle_axis_to_quaternion(math.pi/2, (1, 1, 1))
        
        ros_imu = mock_types.mock_ros_imu_data(x, y, z, w)
        ros_imu.linear_acceleration.x = 1.0
        ros_imu.linear_acceleration.y = 2.0
        ros_imu.linear_acceleration.z = 3.0

        actual: IMU = ros_imu_adapter(ros_imu)

        self.assertIsInstance(actual, IMU)
        self.assertEqual(actual.attitude.x, x)
        self.assertEqual(actual.attitude.y, y)
        self.assertEqual(actual.attitude.z, z)
        self.assertEqual(actual.attitude.w, w)

        # TODO test linear acceleration
        # ROS uses ENU?
        # self.assertEqual(actual.acceleration_linear.east, 1.0)
        # self.assertEqual(actual.acceleration_linear.north, 2.0)
        # self.assertEqual(actual.acceleration_linear.down, -3.0)


    def test_compass(self):
        # test ros_compass_heading_adapter
        ros_heading = mock_types.mock_compass_hdg_data(0.5)
        actual = ros_compass_heading_adapter(ros_heading)
        # self.assertIsInstance(actual, EnuYaw) # ??? This doesn't work?
        self.assertIsInstance(actual, float)
        self.assertEqual(actual, 0.5)

    def test_velocity(self):
        x, y, z = 1.0, 2.0, 3.0
        ros_velocity = mock_types.mock_velocity_data(x, y, z)

        actual: NedVelocity = ros_velocity_adapter(ros_velocity)
        east = x
        north = y
        down = -z
        self.assertIsInstance(actual, NedVelocity)
        self.assertEqual(actual.north, north)
        self.assertEqual(actual.east, east)
        self.assertEqual(actual.down, down)

    def test_position(self):
        test_time = 1719504118.6429088
        ros_pos = mock_types.mock_nav_sat_fix_value(1.0, 2.0, 3.0, test_time)
        actual: TimeStampedLlaPosition = ros_position_adapter(ros_pos)
        self.assertIsInstance(actual, TimeStampedLlaPosition)
        self.assertEqual(actual.time, test_time)
        self.assertEqual(actual.latitude, 1.0)
        self.assertEqual(actual.longitude, 2.0)
        self.assertEqual(actual.altitude, 3.0)
        self.assertEqual(actual.datum_ref, DATUM_REFERENCE.ELLIPSOID_WGS84)

    def test_relative_altitude(self):
        rel_alt = 30.0
        ros_data = mock_types.mock_relative_altitude_data(rel_alt)
        actual: RelativeAltitude = ros_relative_altitude_adapter(ros_data)

        # self.assertIsInstance(actual, RelativeAltitude)
        self.assertIsInstance(actual, float)
        self.assertEqual(actual, rel_alt)


if __name__ == '__main__':
    unittest.main()