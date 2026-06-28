import math
import unittest
from unittest.mock import Mock, NonCallableMagicMock, patch

from std_msgs.msg import Header
from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import Mavlink, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool,
    CommandBoolRequest,
    ParamGet,
    ParamGetRequest,
    ParamGetResponse,
    ParamSet,
    ParamSetRequest,
    ParamSetResponse,
    SetMode,
    SetModeRequest
)
from pymavlink.dialects.v20 import common as mavlink2
import rospy

from src.dr_onboard_autonomy.drone import ArdupilotCopterMavrosDrone, Px4CopterMavrosDrone
from dr_onboard_autonomy.mavlink import (
    ComponentId,
    HeartbeatSender,
    MavlinkNode,
    MavlinkSender,
    SystemId
)
from src.dr_onboard_autonomy.models import (
    ALTITUDE_REFERENCE,
    EnuYaw,
    FCUCopterMode,
    Gimbal,
    LlaPosition,
    NedPosition,
    NedVelocity
)


class TestPx4CopterMavrosDrone(unittest.TestCase):
    @patch("src.dr_onboard_autonomy.drone.mavros.HeartbeatSender", spec=HeartbeatSender)
    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def setUp(
            self,
            mock_rospy: NonCallableMagicMock,
            mock_heartbeat_sender: NonCallableMagicMock
        ):
        self.mock_rospy = mock_rospy
        self.mock_heartbeat_sender = mock_heartbeat_sender
        self.mock_gimbal = NonCallableMagicMock(spec=Gimbal)
        self.mock_uav_name = "a test UAV"

        companion_mav_node = MavlinkNode(
            component_id=ComponentId(mavlink2.MAV_TYPE_ONBOARD_CONTROLLER),
            system_id=SystemId(1)
        )

        self.px4_drone = Px4CopterMavrosDrone(
            gimbal=self.mock_gimbal,
            mavlink_node=companion_mav_node,
            uav_name=self.mock_uav_name
        )


    def test_px4_drone_init(self):
        from src.dr_onboard_autonomy.drone.mavros import _SERVICE_PROXY_TIMEOUT

        self.assertEqual(self.px4_drone.gimbal, self.mock_gimbal)
        self.assertEqual(self.px4_drone.uav_name, self.mock_uav_name)

        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/param/set",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/param/get",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/set_mode",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/cmd/arming",
            _SERVICE_PROXY_TIMEOUT
        )

        self.mock_rospy.ServiceProxy.assert_any_call("mavros/param/set", ParamSet)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/param/get", ParamGet)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/set_mode", SetMode)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/cmd/arming", CommandBool)

        self.mock_rospy.Publisher.assert_any_call(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self.mock_rospy.Publisher.assert_any_call(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=1
        )
        self.mock_rospy.Publisher.assert_any_call(
            "mavlink/to", Mavlink, queue_size=1
        )

        # check simulated GCS heartbeat sender
        self.assertIsInstance(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["mavlink_sender"],
            MavlinkSender
        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["component_type"],
            mavlink2.MAV_TYPE_GCS,

        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["send_frequency"],
            2
        )

        # check companion controller hearbeat sender
        self.assertIsInstance(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["mavlink_sender"],
            MavlinkSender
        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["component_type"],
            mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,

        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["send_frequency"],
            1
        )

        # checks for both heartbeat senders
        self.assertEqual(self.mock_heartbeat_sender.return_value.start.call_count, 2)
        self.assertEqual(
            self.mock_heartbeat_sender.return_value.mav_state,
            mavlink2.MAV_STATE_ACTIVE
        )


    def test_px4_drone_init_service_proxy_hits_timeout(self):
        pass


    def test_get_fcu_parameter_integer(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True
        # integer value non-zero, but float not. Assumption is only one or the other will have a
        # non-zero value
        mock_param_get_response.value.integer = 5

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.px4_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.px4_drone.get_fcu_parameter(name="some px4 param")

        mock_param_get_sp.assert_called_once_with(ParamGetRequest(param_id="some px4 param"))
        self.assertEqual(parameter_value, 5)


    def test_get_fcu_parameter_float(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True
        # real value non-zero, but integer not. Assumption is only one or the other will have a
        # non-zero value
        mock_param_get_response.value.real = 8.76

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.px4_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.px4_drone.get_fcu_parameter(name="some float px4 param")

        self.assertEqual(parameter_value, 8.76)


    def test_get_fcu_parameter_zero(self):
        """The integer zero value returned if both the integer and float response values are zero
        """
        # ParamGetResponse defaults both the integer and real values to zero
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.px4_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.px4_drone.get_fcu_parameter(name="some zero px4 param")

        self.assertEqual(parameter_value, 0)


    def test_get_fcu_parameter_not_successful(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = False

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.px4_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.px4_drone.get_fcu_parameter(name="some unsuccessful px4 param")

        self.assertIsNone(parameter_value)


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_get_fcu_parameter_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_param_get_sp = Mock()
        mock_param_get_sp.side_effect = rospy.ServiceException

        self.px4_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.px4_drone.get_fcu_parameter(name="some px4 param exception")

        self.assertIsNone(parameter_value)


    def test_set_fcu_parameter_integer(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = True

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.px4_drone._param_set_sp = mock_param_set_sp

        self.assertTrue(self.px4_drone.set_fcu_parameter(
            name="set some px4 param",
            value=17
        ))
        mock_param_set_sp.assert_called_once_with(ParamSetRequest(
            param_id="set some px4 param",
            value=ParamValue(
                integer=17,
                real=0.0
            )
        ))


    def test_set_fcu_parameter_float(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = True

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.px4_drone._param_set_sp = mock_param_set_sp

        self.assertTrue(self.px4_drone.set_fcu_parameter(
            name="set some float px4 param",
            value=54.98
        ))
        mock_param_set_sp.assert_called_once_with(ParamSetRequest(
            param_id="set some float px4 param",
            value=ParamValue(
                integer=0,
                real=54.98
            )
        ))


    def test_set_fcu_parameter_not_successful(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = False

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.px4_drone._param_set_sp = mock_param_set_sp

        self.assertFalse(self.px4_drone.set_fcu_parameter(
            name="set some unsuccessful px4 param",
            value=7
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_set_fcu_parameter_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_param_set_sp = Mock()
        mock_param_set_sp.side_effect = rospy.ServiceException
        self.px4_drone._param_set_sp = mock_param_set_sp

        self.assertFalse(self.px4_drone.set_fcu_parameter(
            name="set some px4 param and get exception",
            value=19
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_lla_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717615950)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_global_pub = Mock()
        self.px4_drone._setpoint_global_pub = mock_setpoint_global_pub

        # giving a position with ellipsoid altitude reference to verify that send_lla_setpoint
        # converts to amsl before publishing to Mavros
        lla_position = LlaPosition(34.232, -135.233, 523, ALTITUDE_REFERENCE.ELLIPSOID_WGS84)
        yaw = EnuYaw(33.2 * math.pi / 180)

        self.px4_drone.send_lla_setpoint(lla_position, yaw)

        lla_position.to_amsl()
        expected_geo_pose = GeoPose()
        expected_geo_pose.position.latitude = lla_position.latitude
        expected_geo_pose.position.longitude = lla_position.longitude
        expected_geo_pose.position.altitude = lla_position.altitude

        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, yaw)
        expected_geo_pose.orientation.w = q[3]
        expected_geo_pose.orientation.x = q[0]
        expected_geo_pose.orientation.y = q[1]
        expected_geo_pose.orientation.z = q[2]

        mock_setpoint_global_pub.publish.assert_called_once_with(
            GeoPoseStamped(
                Header(stamp=mock_ros_time),
                expected_geo_pose
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_ned_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717626513)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_local_pub = Mock()
        self.px4_drone._setpoint_local_pub = mock_setpoint_local_pub

        ned_position = NedPosition(256.7, 894.5, -23.6)
        yaw = EnuYaw(47.25 * math.pi / 180)

        self.px4_drone.send_ned_setpoint(ned_position, yaw)

        mock_setpoint_local_pub.publish.assert_called_once_with(
            PositionTarget(
                header=Header(stamp=mock_ros_time),
                coordinate_frame=1,
                type_mask=0b111_111_111_000,
                position=Point(
                    x=ned_position.north,
                    y=ned_position.east,
                    z=ned_position.down
                ),
                yaw=yaw
            )
        )


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_ned_velocity_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717626856)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_local_pub = Mock()
        self.px4_drone._setpoint_local_pub = mock_setpoint_local_pub

        ned_velocity = NedVelocity(10.2, -7.6, 1.1)
        yaw = EnuYaw(125.32 * math.pi / 180)

        self.px4_drone.send_ned_velocity_setpoint(ned_velocity, yaw)

        mock_setpoint_local_pub.publish.assert_called_once_with(
            PositionTarget(
                header=Header(stamp=mock_ros_time),
                coordinate_frame=1,
                type_mask=0b111_111_000_111,
                velocity=Vector3(
                    x=ned_velocity.north,
                    y=ned_velocity.east,
                    z=ned_velocity.down
                ),
                yaw=yaw
            )
        )


    def test_get_available_fcu_modes(self):
        modes = self.px4_drone.get_available_fcu_modes()

        for expected_mode in (
            FCUCopterMode.ALT_HOLD,
            FCUCopterMode.LAND,
            FCUCopterMode.LOITER,
            FCUCopterMode.MISSION,
            FCUCopterMode.OFFBOARD,
            FCUCopterMode.POS_HOLD,
            FCUCopterMode.RTL,
            FCUCopterMode.STABILIZE,
            FCUCopterMode.TAKEOFF
        ):
            self.assertTrue(expected_mode.value in (mode.value for mode in modes))


    def test_set_fcu_mode(self):
        mock_set_mode_sp = Mock(spec=SetMode)
        self.px4_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.return_value.mode_sent = True

        for mode_set in (
            (FCUCopterMode.ALT_HOLD, "ALTCTL"),
            (FCUCopterMode.LAND, "AUTO.LAND"),
            (FCUCopterMode.LOITER, "AUTO.LOITER"),
            (FCUCopterMode.MISSION, "AUTO.MISSION"),
            (FCUCopterMode.OFFBOARD, "OFFBOARD"),
            (FCUCopterMode.POS_HOLD, "POSCTL"),
            (FCUCopterMode.RTL, "AUTO.RTL"),
            (FCUCopterMode.STABILIZE, "STABILIZED"),
            (FCUCopterMode.TAKEOFF, "AUTO.TAKEOFF")
        ):
            with self.subTest(msg=f"flight mode: {mode_set[0].name}"):
                self.assertTrue(self.px4_drone._set_fcu_mode(mode_set[0]))

                self.px4_drone._set_mode_sp.assert_any_call(SetModeRequest(
                    base_mode=None, # use default of 0 when providing a custom mode
                    custom_mode=mode_set[1]
                ))


    def test_set_fcu_mode_sent_false(self):
        mock_set_mode_sp = Mock(spec=SetMode)
        self.px4_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.return_value.mode_sent = False

        self.assertFalse(self.px4_drone._set_fcu_mode(FCUCopterMode.ALT_HOLD))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_set_fcu_mode_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_set_mode_sp = Mock(spec=SetMode)
        self.px4_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.side_effect = rospy.ServiceException

        self.assertFalse(self.px4_drone._set_fcu_mode(FCUCopterMode.ALT_HOLD))


    def test_arm(self):
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.return_value.success = True
        self.px4_drone._arm_sp = mock_arm_sp

        self.assertTrue(self.px4_drone.arm())
        mock_arm_sp.assert_called_once_with(CommandBoolRequest(
            value=True
        ))


    def test_arm_unsucessful(self):
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.return_value.success = False
        self.px4_drone._arm_sp = mock_arm_sp

        self.assertFalse(self.px4_drone.arm())


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_arm_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.side_effect = rospy.ServiceException
        self.px4_drone._arm_sp = mock_arm_sp

        self.assertFalse(self.px4_drone.arm())




class TestArdupilotCopterMavrosDrone(unittest.TestCase):
    @patch("src.dr_onboard_autonomy.drone.mavros.HeartbeatSender", spec=HeartbeatSender)
    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def setUp(
            self,
            mock_rospy: NonCallableMagicMock,
            mock_heartbeat_sender: NonCallableMagicMock
        ):
        self.mock_rospy = mock_rospy
        self.mock_heartbeat_sender = mock_heartbeat_sender
        self.mock_gimbal = NonCallableMagicMock(spec=Gimbal)
        self.mock_uav_name = "a test UAV"

        companion_mav_node = MavlinkNode(
            component_id=ComponentId(mavlink2.MAV_TYPE_ONBOARD_CONTROLLER),
            system_id=SystemId(1)
        )

        self.ardupilot_drone = ArdupilotCopterMavrosDrone(
            gimbal=self.mock_gimbal,
            mavlink_node=companion_mav_node,
            uav_name=self.mock_uav_name
        )


    def test_ardupilot_drone_init(self):
        from src.dr_onboard_autonomy.drone.mavros import _SERVICE_PROXY_TIMEOUT

        self.assertEqual(self.ardupilot_drone.gimbal, self.mock_gimbal)
        self.assertEqual(self.ardupilot_drone.uav_name, self.mock_uav_name)

        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/param/set",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/param/get",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/set_mode",
            _SERVICE_PROXY_TIMEOUT
        )
        self.mock_rospy.wait_for_service.assert_any_call(
            "mavros/cmd/arming",
            _SERVICE_PROXY_TIMEOUT
        )

        self.mock_rospy.ServiceProxy.assert_any_call("mavros/param/set", ParamSet)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/param/get", ParamGet)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/set_mode", SetMode)
        self.mock_rospy.ServiceProxy.assert_any_call("mavros/cmd/arming", CommandBool)

        self.mock_rospy.Publisher.assert_any_call(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self.mock_rospy.Publisher.assert_any_call(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=1
        )
        self.mock_rospy.Publisher.assert_any_call(
            "mavlink/to", Mavlink, queue_size=1
        )

        # check simulated GCS heartbeat sender
        self.assertIsInstance(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["mavlink_sender"],
            MavlinkSender
        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["component_type"],
            mavlink2.MAV_TYPE_GCS,

        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[0].kwargs["send_frequency"],
            2
        )

        # check companion controller hearbeat sender
        self.assertIsInstance(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["mavlink_sender"],
            MavlinkSender
        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["component_type"],
            mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,

        )
        self.assertEqual(
            self.mock_heartbeat_sender.call_args_list[1].kwargs["send_frequency"],
            1
        )

        # checks for both heartbeat senders
        self.assertEqual(self.mock_heartbeat_sender.return_value.start.call_count, 2)
        self.assertEqual(
            self.mock_heartbeat_sender.return_value.mav_state,
            mavlink2.MAV_STATE_ACTIVE
        )

        self.assertIsInstance(self.ardupilot_drone._mavlink_to_fcu, MavlinkSender)
        self.assertEqual(
            self.ardupilot_drone._mavlink_to_fcu.system_id,
            self.ardupilot_drone.mavlink_node.system_id
        )
        self.assertEqual(
            self.ardupilot_drone._mavlink_to_fcu.component_id,
            self.ardupilot_drone.mavlink_node.component_id
        )
        self.assertEqual(
            self.ardupilot_drone._mavlink_to_fcu.mavlink_pub,
            self.ardupilot_drone._mavlink_pub
        )


    def test_mavlink_messages_to_request(self):
        expected_mavlink_messages = (
            mavlink2.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavlink2.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
            mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET,
            mavlink2.MAVLINK_MSG_ID_SYS_STATUS,
            mavlink2.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
            mavlink2.MAVLINK_MSG_ID_BATTERY_STATUS,
            mavlink2.MAVLINK_MSG_ID_ESTIMATOR_STATUS,
            mavlink2.MAVLINK_MSG_ID_ATTITUDE,
            mavlink2.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            mavlink2.MAVLINK_MSG_ID_HIGHRES_IMU,
            mavlink2.MAVLINK_MSG_ID_RAW_IMU,
            mavlink2.MAVLINK_MSG_ID_SCALED_IMU,
            mavlink2.MAVLINK_MSG_ID_SCALED_PRESSURE,
            mavlink2.MAVLINK_MSG_ID_RC_CHANNELS_RAW,
            mavlink2.MAVLINK_MSG_ID_RC_CHANNELS,
            mavlink2.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
        )

        mavlink_messages_requested = (
            msg["msg_id"] for msg in self.ardupilot_drone.FCU_MESSAGES_TO_REQUEST
        )

        for mav_msg_id in expected_mavlink_messages:
            self.assertIn(mav_msg_id, mavlink_messages_requested)


    def test_start(self):
        mock_mavlink_sender_to_fcu = Mock()
        self.ardupilot_drone._mavlink_to_fcu = mock_mavlink_sender_to_fcu

        self.ardupilot_drone.start()

        for mavlink_msg in self.ardupilot_drone.FCU_MESSAGES_TO_REQUEST:
            mock_mavlink_sender_to_fcu.send.assert_any_call(
                mavlink2.MAVLink_command_long_message(
                    target_system=self.ardupilot_drone.mavlink_node.system_id,
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
                ))


    def test_get_fcu_parameter_integer(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True
        # integer value non-zero, but float not. Assumption is only one or the other will have a
        # non-zero value
        mock_param_get_response.value.integer = 5

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.ardupilot_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.ardupilot_drone.get_fcu_parameter(name="some ardupilot param")

        mock_param_get_sp.assert_called_once_with(ParamGetRequest(param_id="some ardupilot param"))
        self.assertEqual(parameter_value, 5)


    def test_get_fcu_parameter_float(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True
        # real value non-zero, but integer not. Assumption is only one or the other will have a
        # non-zero value
        mock_param_get_response.value.real = 8.76

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.ardupilot_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.ardupilot_drone.get_fcu_parameter(name="some float ardupilot param")

        self.assertEqual(parameter_value, 8.76)


    def test_get_fcu_parameter_zero(self):
        """The integer zero value returned if both the integer and float response values are zero
        """
        # ParamGetResponse defaults both the integer and real values to zero
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = True

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.ardupilot_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.ardupilot_drone.get_fcu_parameter(name="some zero ardupilot param")

        self.assertEqual(parameter_value, 0)


    def test_get_fcu_parameter_not_successful(self):
        mock_param_get_response = ParamGetResponse()
        mock_param_get_response.success = False

        mock_param_get_sp = Mock()
        mock_param_get_sp.return_value = mock_param_get_response
        self.ardupilot_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.ardupilot_drone.get_fcu_parameter(
            name="some unsuccessful ardupilot param"
        )

        self.assertIsNone(parameter_value)

    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_get_fcu_parameter_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_param_get_sp = Mock()
        mock_param_get_sp.side_effect = rospy.ServiceException

        self.ardupilot_drone._param_get_sp = mock_param_get_sp

        parameter_value = self.ardupilot_drone.get_fcu_parameter(
            name="some ardupilot param exception"
        )

        self.assertIsNone(parameter_value)


    def test_set_fcu_parameter_integer(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = True

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.ardupilot_drone._param_set_sp = mock_param_set_sp

        self.assertTrue(self.ardupilot_drone.set_fcu_parameter(
            name="set some ardupilot param",
            value=17
        ))
        mock_param_set_sp.assert_called_once_with(ParamSetRequest(
            param_id="set some ardupilot param",
            value=ParamValue(
                integer=17,
                real=0.0
            )
        ))


    def test_set_fcu_parameter_float(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = True

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.ardupilot_drone._param_set_sp = mock_param_set_sp

        self.assertTrue(self.ardupilot_drone.set_fcu_parameter(
            name="set some float ardupilot param",
            value=54.98
        ))
        mock_param_set_sp.assert_called_once_with(ParamSetRequest(
            param_id="set some float ardupilot param",
            value=ParamValue(
                integer=0,
                real=54.98
            )
        ))


    def test_set_fcu_parameter_not_successful(self):
        mock_param_set_response = ParamSetResponse()
        mock_param_set_response.success = False

        mock_param_set_sp = Mock()
        mock_param_set_sp.return_value = mock_param_set_response
        self.ardupilot_drone._param_set_sp = mock_param_set_sp

        self.assertFalse(self.ardupilot_drone.set_fcu_parameter(
            name="set some unsuccessful ardupilot param",
            value=7
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_set_fcu_parameter_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_param_set_sp = Mock()
        mock_param_set_sp.side_effect = rospy.ServiceException
        self.ardupilot_drone._param_set_sp = mock_param_set_sp

        self.assertFalse(self.ardupilot_drone.set_fcu_parameter(
            name="set some ardupilot param and get exception",
            value=19
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_lla_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717615950)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_global_pub = Mock()
        self.ardupilot_drone._setpoint_global_pub = mock_setpoint_global_pub

        # giving a position with ellipsoid altitude reference to verify that send_lla_setpoint
        # converts to amsl before publishing to Mavros
        lla_position = LlaPosition(34.232, -135.233, 523, ALTITUDE_REFERENCE.ELLIPSOID_WGS84)
        yaw = EnuYaw(33.2 * math.pi / 180)

        self.ardupilot_drone.send_lla_setpoint(lla_position, yaw)

        lla_position.to_amsl()
        expected_geo_pose = GeoPose()
        expected_geo_pose.position.latitude = lla_position.latitude
        expected_geo_pose.position.longitude = lla_position.longitude
        expected_geo_pose.position.altitude = lla_position.altitude

        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, yaw)
        expected_geo_pose.orientation.w = q[3]
        expected_geo_pose.orientation.x = q[0]
        expected_geo_pose.orientation.y = q[1]
        expected_geo_pose.orientation.z = q[2]

        mock_setpoint_global_pub.publish.assert_called_once_with(
            GeoPoseStamped(
                Header(stamp=mock_ros_time),
                expected_geo_pose
        ))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_ned_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717626513)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_local_pub = Mock()
        self.ardupilot_drone._setpoint_local_pub = mock_setpoint_local_pub

        ned_position = NedPosition(256.7, 894.5, -23.6)
        yaw = EnuYaw(47.25 * math.pi / 180)

        self.ardupilot_drone.send_ned_setpoint(ned_position, yaw)

        mock_setpoint_local_pub.publish.assert_called_once_with(
            PositionTarget(
                header=Header(stamp=mock_ros_time),
                coordinate_frame=1,
                type_mask=0b111_111_111_000,
                position=Point(
                    x=ned_position.north,
                    y=ned_position.east,
                    z=ned_position.down
                ),
                yaw=yaw
            )
        )


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_send_ned_velocity_setpoint(self, mock_rospy: NonCallableMagicMock):
        mock_ros_time = rospy.rostime.Time(secs=1717626856)
        mock_rospy.Time.now.return_value = mock_ros_time
        mock_setpoint_local_pub = Mock()
        self.ardupilot_drone._setpoint_local_pub = mock_setpoint_local_pub

        ned_velocity = NedVelocity(10.2, -7.6, 1.1)
        yaw = EnuYaw(125.32 * math.pi / 180)

        self.ardupilot_drone.send_ned_velocity_setpoint(ned_velocity, yaw)

        mock_setpoint_local_pub.publish.assert_called_once_with(
            PositionTarget(
                header=Header(stamp=mock_ros_time),
                coordinate_frame=1,
                type_mask=0b111_111_000_111,
                velocity=Vector3(
                    x=ned_velocity.north,
                    y=ned_velocity.east,
                    z=ned_velocity.down
                ),
                yaw=yaw
            )
        )


    def test_get_available_fcu_modes(self):
        modes = self.ardupilot_drone.get_available_fcu_modes()
        available_fcu_modes = (mode.value for mode in modes)

        for expected_mode in (
            FCUCopterMode.ALT_HOLD,
            FCUCopterMode.LAND,
            FCUCopterMode.LOITER,
            FCUCopterMode.MISSION,
            FCUCopterMode.OFFBOARD,
            FCUCopterMode.POS_HOLD,
            FCUCopterMode.RTL,
            FCUCopterMode.STABILIZE
        ):
            self.assertTrue(expected_mode.value in available_fcu_modes)

        # no copter takeoff mode for Ardupilot
        self.assertFalse(FCUCopterMode.TAKEOFF in available_fcu_modes)


    def test_set_fcu_mode(self):
        mock_set_mode_sp = Mock(spec=SetMode)
        self.ardupilot_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.return_value.mode_sent = True

        # http://wiki.ros.org/mavros/CustomModes
        for mode_set in (
            (FCUCopterMode.ALT_HOLD, "ALT_HOLD"),
            (FCUCopterMode.LAND, "LAND"),
            (FCUCopterMode.LOITER, "LOITER"),
            (FCUCopterMode.MISSION, "AUTO"),
            (FCUCopterMode.OFFBOARD, "GUIDED"),
            (FCUCopterMode.POS_HOLD, "POSHOLD"),
            (FCUCopterMode.RTL, "RTL"),
            (FCUCopterMode.STABILIZE, "STABILIZE")
        ):
            with self.subTest(msg=f"flight mode: {mode_set[0].name}"):
                self.assertTrue(self.ardupilot_drone._set_fcu_mode(mode_set[0]))

                self.ardupilot_drone._set_mode_sp.assert_any_call(SetModeRequest(
                    base_mode=None, # use default of 0 when providing a custom mode
                    custom_mode=mode_set[1]
                ))


    def test_set_fcu_mode_sent_false(self):
        mock_set_mode_sp = Mock(spec=SetMode)
        self.ardupilot_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.return_value.mode_sent = False

        self.assertFalse(self.ardupilot_drone._set_fcu_mode(FCUCopterMode.ALT_HOLD))


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_set_fcu_mode_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_set_mode_sp = Mock(spec=SetMode)
        self.ardupilot_drone._set_mode_sp = mock_set_mode_sp
        mock_set_mode_sp.side_effect = rospy.ServiceException

        self.assertFalse(self.ardupilot_drone._set_fcu_mode(FCUCopterMode.ALT_HOLD))


    def test_arm(self):
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.return_value.success = True
        self.ardupilot_drone._arm_sp = mock_arm_sp

        self.assertTrue(self.ardupilot_drone.arm())
        mock_arm_sp.assert_called_once_with(CommandBoolRequest(
            value=True
        ))


    def test_arm_unsucessful(self):
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.return_value.success = False
        self.ardupilot_drone._arm_sp = mock_arm_sp

        self.assertFalse(self.ardupilot_drone.arm())


    @patch("src.dr_onboard_autonomy.drone.mavros.rospy", spec=rospy)
    def test_arm_service_exception(self, mock_rospy):
        mock_rospy.ServiceException = rospy.ServiceException
        mock_arm_sp = Mock(spec=CommandBool)
        mock_arm_sp.side_effect = rospy.ServiceException
        self.ardupilot_drone._arm_sp = mock_arm_sp

        self.assertFalse(self.ardupilot_drone.arm())

