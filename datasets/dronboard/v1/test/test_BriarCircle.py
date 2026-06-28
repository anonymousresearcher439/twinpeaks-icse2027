from droneresponse_mathtools import Lla
import sys
import unittest
from unittest.mock import NonCallableMock, patch

from dr_onboard_autonomy.air_lease import make_buffer_air_tunnel_func, make_circle_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import amsl_to_ellipsoid, BriarLla, convert_tuple_to_LlaDict
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders

from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.trajectory import CircleTrajectory, TrajectoryGenerator

from .mock_types import mock_drone, mock_position_message



class TestBriarCircle(unittest.TestCase):
    def setUp(self):
        self.drone = mock_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)


    
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "CircleTrajectory",
    spec=CircleTrajectory)
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "Gimbal")
    def test_BriarCircle_execute_loop_until_succeeded(
        self,
        mock_gimbal,
        mock_circle_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarCircle

        mock_center_position = convert_tuple_to_LlaDict((39.747377, -105.048213, 5280.2481))
        mock_stare_position = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_sweep_angle = 170
        mock_speed = 4.3
        mock_userdata = NonCallableMock()

        '''
        set briar_circle.trajectory.setpoint_driver.lla with amsl tuple
        '''
        mock_trajectory_instance = NonCallableMock()
        mock_trajectory_instance.configure_mock(**{"setpoint_driver": NonCallableMock()})
        mock_trajectory_instance.setpoint_driver.configure_mock(**{
            "lla": Lla(*(39.747715, -105.048458, 5280.22))
        })
        mock_circle_trajectory.return_value = mock_trajectory_instance
        mock_air_leaser.return_value.execute.side_effect = ("succeeded", "succeeded")

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        briar_circle = BriarCircle(
            center_position=mock_center_position,
            stare_position=mock_stare_position,
            sweep_angle=mock_sweep_angle,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_position_message(*amsl_to_ellipsoid((39.744704, -105.036727, 5279.15))),
            mock_position_message(*amsl_to_ellipsoid((39.747196, -105.045043, 5281.57))),
            mock_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
            mock_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
        ]

        for message in position_messages:
            briar_circle.message_queue.put(message)

        outcome = briar_circle.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_circle")

        mock_circle_trajectory.return_value.start.assert_called_once_with()
        mock_circle_trajectory.return_value.fly_circle.assert_called_once_with(
            BriarLla(mock_center_position, is_amsl=True).amsl.lla,
            mock_sweep_angle,
            mock_speed
        )
        mock_gimbal.return_value.start.assert_called_once_with()

        air_leaser_call_list = mock_air_leaser.call_args_list
        self.assertEqual(
            air_leaser_call_list[0].kwargs["tunnel_func"].__name__,
            make_circle_air_tunnel_func(center_pos=BriarLla(mock_center_position).ellipsoid.lla).__name__
        )
        self.assertEqual(
            air_leaser_call_list[1].kwargs["tunnel_func"].__name__,
            make_buffer_air_tunnel_func().__name__
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "CircleTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "Gimbal")
    def test_BriarCircle_circle_air_lease_failure(
        self,
        mock_gimbal,
        mock_circle_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarCircle
        mock_center_position = convert_tuple_to_LlaDict((39.747377, -105.048213, 5280.2481))
        mock_stare_position = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_userdata = NonCallableMock()

        air_leaser_failure = "some air leaser failure"
        mock_air_leaser.return_value.execute.return_value = air_leaser_failure

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        briar_circle = BriarCircle(
            center_position=mock_center_position,
            stare_position=mock_stare_position,
            **kwargs
        )

        outcome = briar_circle.execute(userdata=mock_userdata)

        self.assertEqual(outcome, air_leaser_failure)


    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "CircleTrajectory",
    spec=CircleTrajectory)
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarCircle"], "Gimbal")
    def test_BriarCircle_buffer_air_lease_failure(
        self,
        mock_gimbal,
        mock_circle_trajectory,
        mock_air_leaser
    ):
        from dr_onboard_autonomy.states import BriarCircle

        mock_center_position = convert_tuple_to_LlaDict((39.747377, -105.048213, 5280.2481))
        mock_stare_position = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        mock_sweep_angle = 170
        mock_speed = 4.3
        mock_userdata = NonCallableMock()

        '''
        set briar_circle.trajectory.setpoint_driver.lla with amsl tuple
        '''
        mock_trajectory_instance = NonCallableMock()
        mock_trajectory_instance.configure_mock(**{"setpoint_driver": NonCallableMock()})
        mock_trajectory_instance.setpoint_driver.configure_mock(**{
            "lla": Lla(*(39.747715, -105.048458, 5280.22))
        })
        mock_circle_trajectory.return_value = mock_trajectory_instance
        air_lease_failure = "some air lease failure"
        mock_air_leaser.return_value.execute.side_effect = ("succeeded", air_lease_failure)

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        briar_circle = BriarCircle(
            center_position=mock_center_position,
            stare_position=mock_stare_position,
            sweep_angle=mock_sweep_angle,
            speed=mock_speed,
            **kwargs
        )

        position_messages = [
            mock_position_message(*amsl_to_ellipsoid((39.744704, -105.036727, 5279.15))),
            mock_position_message(*amsl_to_ellipsoid((39.747196, -105.045043, 5281.57))),
            mock_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
            mock_position_message(*amsl_to_ellipsoid((39.747715, -105.048458, 5280.22))),
        ]

        for message in position_messages:
            briar_circle.message_queue.put(message)

        outcome = briar_circle.execute(userdata=mock_userdata)

        self.assertEqual(outcome, air_lease_failure)

