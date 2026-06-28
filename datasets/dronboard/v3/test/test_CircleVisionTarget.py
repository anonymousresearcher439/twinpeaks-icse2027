from droneresponse_mathtools import Lla
import json
from paho.mqtt.client import MQTTMessage
import sys
import unittest
from unittest.mock import Mock, NonCallableMock, patch

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders
)
from dr_onboard_autonomy.models import (
    DATUM_REFERENCE,
    LlaPosition,
    Quaternion,
    TimeStampedLlaPosition
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.Setpoint import SetpointType

from .mock_types import mock_message_sender_position_message, mock_copter_drone


class TestCircleVisionTarget(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarWaypoint")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    def test_CircleVisionTarget_loop_until_succeeded(
        self,
        mock_briar_hover,
        mock_briar_waypoint,
        mock_briar_circle
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        mock_takeoff_location = TimeStampedLlaPosition(
            0.0,
            0.0,
            0.0,
            DATUM_REFERENCE.ELLIPSOID_WGS84,
            time=12.34
        )
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data" : {
                "arm_position" : mock_takeoff_location,
                'setpoint_type': SetpointType.LLA,
                'setpoint_value': (39.744704, -105.036727, 5287.15),
                'setpoint_yaw': 3.14,
            }
        }
        self.drone.data.arm_position = mock_takeoff_location

        mock_userdata = NonCallableMock()
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"
        mock_briar_waypoint.return_value.execute.return_value = "succeeded_waypoints"
        mock_briar_circle.return_value.execute.return_value = "succeeded_circle"
        mock_target_approach_speed = 6.5
        mock_target_circle_speed = 3.3

        circle_vision_target = CircleVisionTarget(
            target_circle_radius=10.0,
            target_circle_height=5.0,
            target_approach_speed=mock_target_approach_speed,
            circle_speed=mock_target_circle_speed,
            **kwargs
        )

        mock_vision_MQTTMessage = NonCallableMock(spec=MQTTMessage)
        mock_vision_MQTTMessage.payload = json.dumps({
            "x" : 200,
            "y" : 200,
            "x_res" : 400,
            "y_res" : 400,
            "ts" : 12345.0
        }).encode()
        vision_message = {"type": "vision", "data": mock_vision_MQTTMessage}
        circle_vision_target._read_messages.message_queue.put(vision_message)
        position_message = LlaPosition(0.0, 0.0, 50.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        self.drone.data.location.lookup_position = Mock(return_value=position_message)
        self.drone.data.location.add_position(12345.0 + 1, position_message)

        drone_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.drone.data.attitude.lookup_attitude = Mock(return_value=drone_attitude)
        self.drone.data.attitude.add_attitude(12345.0 + 1, drone_attitude)

        gimbal_attitude = Quaternion(0.0, 0.3826834, 0.0, 0.9238795)
        self.drone.gimbal.attitude.lookup_attitude = Mock(return_value=gimbal_attitude)
        self.drone.gimbal.attitude.add_attitude(12345.0 + 1, gimbal_attitude)

        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_circle")

        '''target_position published on global mqtt once determined
        '''
        publish_target_position_args = self.mock_mqtt.publish.call_args.args
        self.assertEqual(publish_target_position_args[0], "target_position")

        publish_target_position_values = publish_target_position_args[1]
        self.assertAlmostEqual(publish_target_position_values["latitude"], 0.0)
        self.assertAlmostEqual(publish_target_position_values["longitude"], 0.0004491576420505598)
        self.assertAlmostEqual(
            publish_target_position_values["altitude_amsl"],
            -17.162671608080853
        )

        expected_pass_through_kwargs = kwargs

        expected_pass_through_kwargs["outcomes"] = [
            "succeeded_circle",
            "error",
            "failsafe",
            "abort",
            "rtl"
        ]

        briar_hover_kwargs = mock_briar_hover.call_args.kwargs
        self.assertEqual(
            briar_hover_kwargs["hover_time"],
            5.0
        )
        self.assertTrue(set(expected_pass_through_kwargs).issubset(briar_hover_kwargs))

        briar_waypoint_kwargs = mock_briar_waypoint.call_args.kwargs
        circle_start_pos = BriarLla.from_dict(briar_waypoint_kwargs["waypoint"])
        target_pos = BriarLla.from_args(
            publish_target_position_values["latitude"],
            publish_target_position_values["longitude"],
            publish_target_position_values["altitude_amsl"],
            is_amsl=True
        )
        target_circle_radius=10.0
        target_circle_height=5.0
        center = target_pos.ellipsoid.lla.move_ned(0,0, -target_circle_height)
        # to test that the drone flies to the edge of the circle
        # make sure we fly to a point that's about target_circle_radius meters
        # from the center of the circle that's just above the target_pos
        self.assertAlmostEqual(center.distance(circle_start_pos.ellipsoid.lla), target_circle_radius, places=2)

        self.assertAlmostEqual(
            briar_waypoint_kwargs["stare_position"]["latitude"],
            0.0
        )
        self.assertAlmostEqual(
            briar_waypoint_kwargs["stare_position"]["longitude"],
            0.0004491576420505598
        )
        self.assertAlmostEqual(
            briar_waypoint_kwargs["stare_position"]["altitude"],
            -17.162671608080853,
            3
        )

        self.assertEqual(
            briar_waypoint_kwargs["speed"],
            mock_target_approach_speed
        )
        self.assertTrue(set(expected_pass_through_kwargs).issubset(briar_waypoint_kwargs))

        briar_circle_kwargs = mock_briar_circle.call_args.kwargs
        self.assertAlmostEqual(
            briar_circle_kwargs["center_position"]["latitude"],
            0.0
        )
        self.assertAlmostEqual(
            briar_circle_kwargs["center_position"]["longitude"],
            0.0004491576420505598
        )
        self.assertAlmostEqual(
            briar_circle_kwargs["center_position"]["altitude"],
            -17.162671608080853,
            3
        )

        self.assertAlmostEqual(
            briar_circle_kwargs["stare_position"]["latitude"],
            0.0
        )
        self.assertAlmostEqual(
            briar_circle_kwargs["stare_position"]["longitude"],
            0.0004491576420505598
        )
        self.assertAlmostEqual(
            briar_circle_kwargs["stare_position"]["altitude"],
            -17.162671608080853,
            3
        )

        self.assertEqual(
            briar_circle_kwargs["speed"],
            mock_target_circle_speed
        )
        self.assertTrue(set(expected_pass_through_kwargs).issubset(briar_circle_kwargs))


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "ReadMessagesAirborne")
    def test_CircleVisionTarget_loop_read_messages_failure(
        self,
        mock_read_messages,
        mock_briar_hover
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {}
        }

        mock_userdata = NonCallableMock()
        mock_read_messages_error = "error"
        mock_read_messages.return_value.execute.return_value = mock_read_messages_error
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"

        circle_vision_target = CircleVisionTarget(**kwargs)

        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, mock_read_messages_error)


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")

    def test_CircleVisionTarget_loop_retrieve_position_failure(
        self,
        mock_briar_hover
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {},
        }

        self.drone.data.location.lookup_position = Mock(side_effect=Exception())
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"

        mock_userdata = NonCallableMock()
        mock_vision_MQTTMessage = NonCallableMock(spec=MQTTMessage)
        mock_vision_MQTTMessage.payload = json.dumps({
            "x" : 200,
            "y" : 200,
            "x_res" : 400,
            "y_res" : 400,
            "ts" : 12345.0
        }).encode()
        vision_message = {"type": "vision", "data": mock_vision_MQTTMessage}

        circle_vision_target = CircleVisionTarget(**kwargs)
        circle_vision_target.drone.data.location.last = Mock(return_value=(12345.0 + 1, None))
        circle_vision_target.drone.data.attitude.last = Mock(return_value=(12345.0 + 1, None))

        circle_vision_target._read_messages.message_queue.put(vision_message)
        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "error")


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    def test_CircleVisionTarget_loop_BriarHover_failure(self, mock_briar_hover):
        from dr_onboard_autonomy.states import CircleVisionTarget

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        briar_hover_error = "some error"
        mock_briar_hover.return_value.execute.return_value = briar_hover_error

        circle_vision_target = CircleVisionTarget(**kwargs)

        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, briar_hover_error)


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarWaypoint")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    def test_CircleVisionTarget_loop_BriarWaypoint_failure(
        self,
        mock_briar_hover,
        mock_briar_waypoint
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        mock_takeoff_location = TimeStampedLlaPosition(
            0.0,
            0.0,
            0.0,
            DATUM_REFERENCE.ELLIPSOID_WGS84,
            time=12.34
        )
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data" : {
                "arm_position" : mock_takeoff_location
            }
        }
        self.drone.data.arm_position = mock_takeoff_location

        mock_userdata = NonCallableMock()
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"
        mock_briar_waypoint_failure = "some failure"
        mock_briar_waypoint.return_value.execute.return_value = mock_briar_waypoint_failure

        mock_target_approach_speed = 6.5

        circle_vision_target = CircleVisionTarget(
            target_circle_radius=10.0,
            target_circle_height=5.0,
            target_approach_speed=mock_target_approach_speed,
            circle_speed=3.3,
            **kwargs
        )

        mock_vision_MQTTMessage = NonCallableMock(spec=MQTTMessage)
        mock_vision_MQTTMessage.payload = json.dumps({
            "x" : 200,
            "y" : 200,
            "x_res" : 400,
            "y_res" : 400,
            "ts" : 12345.0
        }).encode()
        vision_message = {"type": "vision", "data": mock_vision_MQTTMessage}
        circle_vision_target._read_messages.message_queue.put(vision_message)

        position_message = LlaPosition(0.0, 0.0, 50.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        self.drone.data.location.lookup_position = Mock(return_value=position_message)
        self.drone.data.location.add_position(12345.0 + 1, position_message)

        drone_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.drone.data.attitude.lookup_attitude = Mock(return_value=drone_attitude)
        self.drone.data.attitude.add_attitude(12345.0 + 1, drone_attitude)
        
        gimbal_attitude = Quaternion(0.0, 0.3826834, 0.0, 0.9238795)
        self.drone.gimbal.attitude.lookup_attitude = Mock(return_value=gimbal_attitude)
        self.drone.gimbal.attitude.add_attitude(12345.0 + 1, gimbal_attitude)

        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, mock_briar_waypoint_failure)


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarWaypoint")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    def test_CircleVisionTarget_loop_BriarCircle_failure(
        self,
        mock_briar_hover,
        mock_briar_waypoint,
        mock_briar_circle
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        mock_takeoff_location = TimeStampedLlaPosition(
            0.0,
            0.0,
            0.0,
            DATUM_REFERENCE.ELLIPSOID_WGS84,
            time=12.34
        )
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data" : {
                "arm_position" : mock_takeoff_location
            }
        }
        self.drone.data.arm_position = mock_takeoff_location

        mock_userdata = NonCallableMock()
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"
        mock_briar_waypoint.return_value.execute.return_value = "succeeded_waypoints"
        mock_briar_circle_failure = "some failure"
        mock_briar_circle.return_value.execute.return_value = mock_briar_circle_failure
        mock_target_approach_speed = 6.5

        circle_vision_target = CircleVisionTarget(
            target_circle_radius=10.0,
            target_circle_height=5.0,
            target_approach_speed=mock_target_approach_speed,
            circle_speed=3.3,
            **kwargs
        )

        mock_vision_MQTTMessage = NonCallableMock(spec=MQTTMessage)
        mock_vision_MQTTMessage.payload = json.dumps({
            "x" : 200,
            "y" : 200,
            "x_res" : 400,
            "y_res" : 400,
            "ts" : 12345.0
        }).encode()
        vision_message = {"type": "vision", "data": mock_vision_MQTTMessage}
        position_message = mock_message_sender_position_message(0.0, 0.0, 50.0)
        circle_vision_target._read_messages.message_queue.put(vision_message)

        position_message = LlaPosition(0.0, 0.0, 50.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        self.drone.data.location.lookup_position = Mock(return_value=position_message)
        self.drone.data.location.add_position(12345.0 + 1, position_message)

        drone_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.drone.data.attitude.lookup_attitude = Mock(return_value=drone_attitude)
        self.drone.data.attitude.last = Mock(return_value=(12345.0 + 1, drone_attitude))
        self.drone.data.attitude.add_attitude(12345.0 + 1, drone_attitude)

        gimbal_attitude = Quaternion(0.0, 0.3826834, 0.0, 0.9238795)
        self.drone.gimbal.attitude.lookup_attitude = Mock(return_value=gimbal_attitude)
        self.drone.gimbal.attitude.add_attitude(12345.0 + 1, gimbal_attitude)

        outcome = circle_vision_target.execute(userdata=mock_userdata)

        self.assertEqual(outcome, mock_briar_circle_failure)


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarWaypoint")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleVisionTarget"], "BriarHover")
    def test_CircleVisionTarget_ts_newer_than_drone_attitude(
        self,
        mock_briar_hover,
        mock_briar_waypoint,
        mock_briar_circle
    ):
        from dr_onboard_autonomy.states import CircleVisionTarget

        mock_takeoff_location = TimeStampedLlaPosition(
            0.0,
            0.0,
            0.0,
            DATUM_REFERENCE.ELLIPSOID_WGS84,
            time=12.34
        )
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data" : {
                "arm_position" : mock_takeoff_location
            }
        }
        self.drone.data.arm_position = mock_takeoff_location

        mock_userdata = NonCallableMock()
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"
        mock_briar_waypoint.return_value.execute.return_value = "succeeded_waypoints"
        mock_briar_circle.return_value.execute.return_value = "succeeded_circle"
        mock_target_approach_speed = 6.5
        mock_target_circle_speed = 3.3

        circle_vision_target = CircleVisionTarget(
            target_circle_radius=10.0,
            target_circle_height=5.0,
            target_approach_speed=mock_target_approach_speed,
            circle_speed=mock_target_circle_speed,
            **kwargs
        )

        circle_vision_target._read_drone_data = Mock()
        circle_vision_target._read_drone_data.execute = Mock(return_value="succeeded")

        mock_vision_MQTTMessage = NonCallableMock(spec=MQTTMessage)
        mock_vision_MQTTMessage.payload = json.dumps({
            "x" : 200,
            "y" : 200,
            "x_res" : 400,
            "y_res" : 400,
            "ts" : 12345.0
        }).encode()
        vision_message = {"type": "vision", "data": mock_vision_MQTTMessage}
        circle_vision_target._read_messages.message_queue.put(vision_message)

        position_message = LlaPosition(0.0, 0.0, 50.0, DATUM_REFERENCE.ELLIPSOID_WGS84)
        self.drone.data.location.lookup_position = Mock(return_value=position_message)
        self.drone.data.location.add_position(12345.0 + 1, position_message)

        drone_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.drone.data.attitude.lookup_attitude = Mock(return_value=drone_attitude)
        self.drone.data.attitude.add_attitude(12345.0 - 1, drone_attitude)

        gimbal_attitude = Quaternion(0.0, 0.3826834, 0.0, 0.9238795)
        self.drone.gimbal.attitude.lookup_attitude = Mock(return_value=gimbal_attitude)
        self.drone.gimbal.attitude.add_attitude(12345.0 + 1, drone_attitude)

        outcome = circle_vision_target.execute(mock_userdata)
        self.assertEqual(outcome, "succeeded_circle")
        circle_vision_target._read_drone_data.execute.assert_called_with(mock_userdata)

