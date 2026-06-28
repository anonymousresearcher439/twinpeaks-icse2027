from droneresponse_mathtools import Lla
import sys
import unittest
from unittest.mock import (
    NonCallableMock,
    patch
)
from math import pi
from queue import Queue


from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    BriarLla,
    convert_tuple_to_LlaDict,
    convert_Lla_to_LlaDict,
)
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator

from .mock_types import mock_message_sender_position_message, mock_copter_drone

class TestBriarWaypoint(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)
    

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    def test_BriarWaypoint_execute_loop_until_succeeded(
        self,
        mock_gimbal,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarWaypoint

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        end_pos_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_speed = 3.4
        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        brair_waypoint_state = BriarWaypoint(
            waypoint=end_pos_LlaDict,
            stare_position=end_pos_LlaDict,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.744704, -105.036727, 5287.15))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747196, -105.045043, 5294.95))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
        ]
        
        for message in position_messages:
            brair_waypoint_state.message_queue.put(message)

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_waypoints")
        mock_waypoint_trajectory.return_value.start.assert_called_once_with()
        mock_gimbal.return_value.start.assert_called_once_with()

        air_leaser_call_kwargs = mock_air_leaser.call_args.kwargs
        self.assertEqual(
            air_leaser_call_kwargs["tunnel_func"].__name__,
            make_waypoint_multi_air_tunnel_func(end_pos=BriarLla(end_pos_LlaDict).ellipsoid.lla).__name__
        )
        mock_air_leaser.return_value.communicate_route_complete.assert_called_once_with(
            current_position=Lla(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22)))
        )
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    def test_BriarWaypoint_execute_loop_until_succeeded_with_pitch_angle(
        self,
        mock_gimbal,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarWaypoint

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        end_pos_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_speed = 3.4
        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        brair_waypoint_state = BriarWaypoint(
            waypoint=end_pos_LlaDict,
            stare_pitch=45,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.744704, -105.036727, 5287.15))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747196, -105.045043, 5294.95))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
        ]
        
        for message in position_messages:
            brair_waypoint_state.message_queue.put(message)

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_waypoints")
        mock_waypoint_trajectory.return_value.start.assert_called_once_with()
        mock_gimbal.return_value.start.assert_called_once_with()

        air_leaser_call_kwargs = mock_air_leaser.call_args.kwargs
        self.assertEqual(
            air_leaser_call_kwargs["tunnel_func"].__name__,
            make_waypoint_multi_air_tunnel_func(end_pos=BriarLla(end_pos_LlaDict).ellipsoid.lla).__name__
        )
        mock_air_leaser.return_value.communicate_route_complete.assert_called_once_with(
            current_position=Lla(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22)))
        )
        actual_q = mock_gimbal.return_value.fixed_direction
        expected_q = [0.0, 0.3826834323650898, 0.0, 0.9238795325112867]
        self.assertEqual(len(actual_q), len(expected_q))
        for actual_elm, expected_elm in zip(actual_q, expected_q):
            self.assertAlmostEqual(actual_elm, expected_elm)
        


    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    def test_BriarWaypoint_execute_air_leaser_failure(
        self,
        mock_gimbal,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarWaypoint

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        end_pos_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_speed = 3.4
        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser_failure = "some failure outcome"
        mock_air_leaser.return_value.execute.return_value = mock_air_leaser_failure

        brair_waypoint_state = BriarWaypoint(
            waypoint=end_pos_LlaDict,
            stare_position=end_pos_LlaDict,
            speed=mock_speed,
            **kwargs
        )

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, mock_air_leaser_failure)
    
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    def test_BriarWaypoint_will_exit_when_within_the_threshold_distance(self, mock_waypoint_trajectory, mock_gimbal, mock_air_leaser):

        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.mavros_layer import MAVROSDrone
        from dr_onboard_autonomy.message_senders import ReusableMessageSenders
        from dr_onboard_autonomy.mqtt_client import MQTTClient
        from dr_onboard_autonomy.states import BriarWaypoint

        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        kwargs = {
            "air_lease_service": NonCallableMock(spec=AirLeaseService),
            "drone": NonCallableMock(),
            "heartbeat_handler": False,
            "local_mqtt_client": NonCallableMock(spec=MQTTClient),
            "mqtt_client": NonCallableMock(spec=MQTTClient),
            "name": "TEST_THRESHOLD_DISTANCE",
            "outcomes": ["found", "human_control", "abort", "success", "error", "rtl"],
            "reusable_message_senders": NonCallableMock(spec=ReusableMessageSenders),
            "uav_id": "uav_id",
        }

        mock_userdata = NonCallableMock()
         #= briarlla_from_lat_lon_alt(39.747714, -105.048456, 5280.2481)
        peppermint_rc_runway = BriarLla({
            "latitude": 41.606683520364925, 
            "longitude": -86.35605373685361,
            "altitude": 229.02
        }, is_amsl=True)
        peppermint_waypoint = BriarLla(convert_Lla_to_LlaDict(peppermint_rc_runway.ellipsoid.lla.move_ned(-10, 30, -25)), is_amsl=False)
        
        mock_speed = 3.4

        brair_waypoint_state = BriarWaypoint(
            waypoint=peppermint_waypoint.amsl.dict,
            stare_position=peppermint_rc_runway.amsl.dict,
            speed=mock_speed,
            **kwargs
        )

        def mock_position_msg(lla: Lla):
            pos = NonCallableMock()
            pos.latitude = lla.latitude
            pos.longitude = lla.longitude
            pos.altitude = lla.altitude
            return {"type": "position", "data": pos}

        for d in range(100, 2, -2):
            drone_pos = mock_position_msg(peppermint_waypoint.ellipsoid.lla.move_ned(d, 0, 0))
            brair_waypoint_state.message_queue.put(drone_pos)
        
        brair_waypoint_state.message_queue.put(mock_position_msg(peppermint_waypoint.ellipsoid.lla.move_ned(1.49999, 0, 0)))
        brair_waypoint_state.message_queue.put({"type": "shutdown", "data": None})

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_waypoints")
    

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    def test_BriarWaypoint_when_outside_threshold_distance(self, mock_waypoint_trajectory, mock_gimbal, mock_air_leaser):

        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.mavros_layer import MAVROSDrone
        from dr_onboard_autonomy.message_senders import ReusableMessageSenders
        from dr_onboard_autonomy.mqtt_client import MQTTClient
        from dr_onboard_autonomy.states import BriarWaypoint

        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        kwargs = {
            "air_lease_service": NonCallableMock(spec=AirLeaseService),
            "drone": NonCallableMock(),
            "heartbeat_handler": False,
            "local_mqtt_client": NonCallableMock(spec=MQTTClient),
            "mqtt_client": NonCallableMock(spec=MQTTClient),
            "name": "TEST_THRESHOLD_DISTANCE",
            "outcomes": ["found", "human_control", "abort", "success", "error", "rtl"],
            "reusable_message_senders": NonCallableMock(spec=ReusableMessageSenders),
            "uav_id": "uav_id",
        }

        mock_userdata = NonCallableMock()
         #= briarlla_from_lat_lon_alt(39.747714, -105.048456, 5280.2481)
        peppermint_rc_runway = BriarLla({
            "latitude": 41.606683520364925, 
            "longitude": -86.35605373685361,
            "altitude": 229.02
        }, is_amsl=True)
        peppermint_waypoint = BriarLla(convert_Lla_to_LlaDict(peppermint_rc_runway.ellipsoid.lla.move_ned(-10, 30, -25)), is_amsl=False)
        
        mock_speed = 3.4

        brair_waypoint_state = BriarWaypoint(
            waypoint=peppermint_waypoint.amsl.dict,
            stare_position=peppermint_rc_runway.amsl.dict,
            speed=mock_speed,
            **kwargs
        )

        def mock_position_msg(lla: Lla):
            pos = NonCallableMock()
            pos.latitude = lla.latitude
            pos.longitude = lla.longitude
            pos.altitude = lla.altitude
            return {"type": "position", "data": pos}

        for d in range(100, 2, -2):
            drone_pos = mock_position_msg(peppermint_waypoint.ellipsoid.lla.move_ned(d, 0, 0))
            brair_waypoint_state.message_queue.put(drone_pos)
        
        brair_waypoint_state.message_queue.put(mock_position_msg(peppermint_waypoint.ellipsoid.lla.move_ned(1.51, 0, 0)))
        brair_waypoint_state.message_queue.put({"type": "shutdown", "data": None})

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "error")

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "Gimbal")
    def test_BriarWaypoint_with_no_stare_position(
        self,
        mock_gimbal,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarWaypoint

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        end_pos_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_speed = 3.4
        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        brair_waypoint_state = BriarWaypoint(
            waypoint=end_pos_LlaDict,
            stare_position=None,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.744704, -105.036727, 5287.15))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747196, -105.045043, 5294.95))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
        ]
        
        for message in position_messages:
            brair_waypoint_state.message_queue.put(message)

        outcome = brair_waypoint_state.execute(userdata=mock_userdata)
        mock_gimbal.return_value.start.assert_not_called()

        self.assertEqual(outcome, "succeeded_waypoints")

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarWaypoint"], "WaypointTrajectory")
    def test_BriarWaypoint_sets_yaw_throughout_flight(
        self,
        mock_waypoint_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarWaypoint

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        end_pos_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_speed = 3.4
        mock_waypoint_trajectory.return_value.is_done.return_value = True
        mock_air_leaser.return_value.execute.return_value = "succeeded"

        briar_waypoint_state = BriarWaypoint(
            waypoint=end_pos_LlaDict,
            stare_position=None,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_message_sender_position_message(*amsl_to_ellipsoid((19.747714, -105.048456, 5280.2481))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((29.747714, -105.048456, 5280.2481))),
            mock_message_sender_position_message(*amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))),
        ]

        test_position_queue = Queue()
        briar_waypoint_state.message_queue.put(position_messages[0])

        for message in position_messages[1:]:
            test_position_queue.put(message)

        def add_next_position_handler(_):
            briar_waypoint_state.message_queue.put(test_position_queue.get())

        briar_waypoint_state.handlers.add_handler("position", add_next_position_handler)

        outcome = briar_waypoint_state.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_waypoints")
        self.assertEqual(round(mock_waypoint_trajectory.return_value.yaw, 10), round(pi / 2.0, 10))
