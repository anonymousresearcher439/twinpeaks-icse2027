from queue import Queue
import sys
import unittest
from unittest.mock import (
    NonCallableMock,
    patch
)

from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    BriarLla,
    convert_tuple_to_LlaDict
)
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator

from .mock_types import mock_message_sender_position_message, mock_copter_drone

class TestTakeoff(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)

    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Offboard")    
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Arm")  
    def test_Takeoff_state_execute_loop_until_message_says_the_drone_is_at_altitude(
        self,
        mock_arm_state,
        mock_read_position_state,
        mock_offboard_state,
        mock_queue,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import Takeoff
        mock_offboard_state.return_value.execute.return_value = "succeeded"
        mock_arm_state.return_value.execute.return_value = "succeeded_armed"
        mock_read_position_state.return_value.execute.return_value = "succeeded"
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_air_leaser.return_value.execute.return_value = "succeeded"

        mock_waypoint_trajectory.return_value.is_done.return_value = True

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {}

        }

        mock_userdata = NonCallableMock()
        mock_takeoff_alt = 15.0

        takeoff_state = Takeoff(altitude=mock_takeoff_alt, **kwargs)
        takeoff_state.message_queue = Queue()

        position_messages = [
            mock_message_sender_position_message(lat=39.747714, lon=-105.048456, alt=5287.15),
            mock_message_sender_position_message(lat=39.747714, lon=-105.048456, alt=5294.95),
            mock_message_sender_position_message(lat=39.747714, lon=-105.048456, alt=5302.54),
        ]

        for message in position_messages:
            takeoff_state.message_queue.put(message)

        takeoff_outcome = takeoff_state.execute(userdata=mock_userdata)
        self.assertEqual(takeoff_outcome, "succeeded_takeoff")

        mock_final_pos = pos_briar_lla.ellipsoid.lla.move_ned(0, 0, -1.0 * mock_takeoff_alt)

        air_leaser_call_kwargs = mock_air_leaser.call_args.kwargs
        self.assertEqual(
            air_leaser_call_kwargs["tunnel_func"].__name__,
            make_waypoint_multi_air_tunnel_func(end_pos=mock_final_pos).__name__
        )
        self.assertFalse(air_leaser_call_kwargs["is_flying"])
        mock_waypoint_trajectory.return_value.start.assert_called_once_with()


    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Arm")
    def test_Takeoff_state_air_leaser_execute_failure(
        self,
        mock_arm_state,
        mock_read_position_state,
        mock_queue,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import Takeoff
        mock_arm_state.return_value.execute.return_value = "succeeded_armed"
        mock_read_position_state.return_value.execute.return_value = "succeeded"
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_air_leaser_failure = "some failure outcome"
        mock_air_leaser.return_value.execute.return_value = "some failure outcome"
        mock_arm_state.return_value.execute.return_value = "succeeded_armed"

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {},
        }

        mock_userdata = NonCallableMock()
        mock_takeoff_alt = 15.0

        takeoff_state = Takeoff(altitude=mock_takeoff_alt, **kwargs)

        takeoff_outcome = takeoff_state.execute(userdata=mock_userdata)

        self.assertEqual(takeoff_outcome, mock_air_leaser_failure)


    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Offboard")    
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Arm")
    def test_Takeoff_state_read_position_execute_failure(
        self,
        mock_arm_state,
        mock_read_position_state,
        mock_offboard_state,
        mock_queue,
        mock_waypoint_trajectory
    ):
        from dr_onboard_autonomy.states import Takeoff
        mock_offboard_state.return_value.execute.return_value = "succeeded"
        mock_arm_state.return_value.execute.return_value = "succeeded_armed"
        mock_read_position_failure = "some failure outcome"
        mock_read_position_state.return_value.execute.return_value = mock_read_position_failure
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_waypoint_trajectory.return_value.is_done.return_value = True

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {},
        }

        mock_userdata = NonCallableMock()
        mock_takeoff_alt = 15.0

        takeoff_state = Takeoff(altitude=mock_takeoff_alt, **kwargs)

        takeoff_outcome = takeoff_state.execute(userdata=mock_userdata)

        self.assertEqual(takeoff_outcome, mock_read_position_failure)

    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Offboard")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Arm")
    def test_Takeoff_state_offboard_execute_failure(
        self,
        mock_arm_state,
        mock_read_position_state,
        mock_offboard_state,
        mock_queue,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import Takeoff
        mock_arm_state.return_value.execute.return_value = "succeeded_armed"
        mock_read_position_state.return_value.execute.return_value = "succeeded"
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        mock_offboard_failure = "some failure outcome"
        mock_offboard_state.return_value.execute.return_value = mock_offboard_failure

        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_waypoint_trajectory.return_value.is_done.return_value = True

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {},
        }

        mock_userdata = NonCallableMock()
        mock_takeoff_alt = 15.0

        takeoff_state = Takeoff(altitude=mock_takeoff_alt, **kwargs)

        takeoff_outcome = takeoff_state.execute(userdata=mock_userdata)

        self.assertEqual(takeoff_outcome, mock_offboard_failure)

    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Offboard")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Takeoff"], "Arm")
    def test_Takeoff_state_arm_state_execute_failure(
        self,
        mock_arm_state,
        mock_read_position_state,
        mock_offboard_state,
        mock_queue,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import Takeoff
        mock_failure_outcome = "error"
        mock_arm_state.return_value.execute.return_value = mock_failure_outcome
        mock_read_position_state.return_value.execute.return_value = "succeeded"
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        
        mock_offboard_state.return_value.execute.return_value = "succeeded"

        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_waypoint_trajectory.return_value.is_done.return_value = True

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {},
        }

        mock_userdata = NonCallableMock()
        mock_takeoff_alt = 15.0

        takeoff_state = Takeoff(altitude=mock_takeoff_alt, **kwargs)

        takeoff_outcome = takeoff_state.execute(userdata=mock_userdata)

        self.assertEqual(takeoff_outcome, mock_failure_outcome)