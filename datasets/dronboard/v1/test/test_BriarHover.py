

import sys
import unittest
from unittest.mock import NonCallableMock, patch

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    convert_LlaDict_to_tuple,
    convert_tuple_to_LlaDict,
    ellipsoid_to_amsl,
    LlaDict,
)
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator

from .mock_types import mock_drone, mock_position_message

class TestBriarHover(unittest.TestCase):
    
    def setUp(self):
        self.drone = mock_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)
    

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "YawTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "Gimbal")
    def test_BriarHover_execute_loop_until_succeeded(
        self,
        mock_gimbal,
        mock_yaw_trajectory
    ):
        from dr_onboard_autonomy.states import BriarHover

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        pos_Lla = Lla(*amsl_to_ellipsoid((39.744704, -105.036727, 5287.15)))
        stare_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        briar_hover = BriarHover(
            hover_time=5.0,
            stare_position=stare_LlaDict,
            **kwargs
        )

        messages = [
            mock_position_message(pos_Lla.lat, pos_Lla.lon, pos_Lla.alt),
            {"type": "hover_timer", "data": 3.0},
            {"type": "hover_timer", "data": 5.5}
        ]

        for message in messages:
            briar_hover.message_queue.put(message)

        outcome = briar_hover.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_gimbal.return_value.start.assert_called_once_with()
        mock_yaw_trajectory.return_value.setpoint_driver.start.assert_called_once_with()
        self.assertEqual(
            mock_gimbal.return_value.stare_position,
            amsl_to_ellipsoid(convert_LlaDict_to_tuple(stare_LlaDict))
        )
        self.assertEqual(
            mock_yaw_trajectory.return_value.setpoint_driver.lla,
            ellipsoid_to_amsl((pos_Lla.lat, pos_Lla.lon, pos_Lla.alt))
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "RepeatTimer")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "YawTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "Gimbal")
    def test_BriarHover_loop_until_succeeded(
        self,
        mock_gimbal,
        mock_yaw_trajectory,
        mock_repeat_timer
    ):
        from dr_onboard_autonomy.states import BriarHover
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        mock_hover_time = 8.2
        mock_stare_position = LlaDict(latitude=39.744704, longitude=-105.036727, altitude=5287.15)
        briar_hover = BriarHover(
            hover_time=mock_hover_time,
            stare_position=mock_stare_position,
            **kwargs
        )

        amsl_pos = (39.744704, -105.036727, 5287.15)
        position_message = mock_position_message(*amsl_to_ellipsoid(amsl_pos))

        hover_timer_message = {"type": "hover_timer", "data": 0.26334}

        briar_hover.message_queue.put(position_message)
        briar_hover.message_queue.put(hover_timer_message)

        outcome = briar_hover.execute(mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_yaw_trajectory.return_value.setpoint_driver.start.assert_called_once_with()
        mock_gimbal.return_value.start.assert_called_once_with()
        mock_repeat_timer.assert_called_once_with("hover_timer", mock_hover_time)
        self.assertEqual(briar_hover.hover_pos, amsl_pos)
    
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "YawTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "Gimbal")
    def test_BriarHover_execute_loop_use_default_stare_position(
        self,
        mock_gimbal,
        mock_yaw_trajectory
    ):
        from dr_onboard_autonomy.states import BriarHover

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        pos_Lla = Lla(*amsl_to_ellipsoid((39.744704, -105.036727, 5287.15)))
        briar_hover = BriarHover(
            hover_time=5.0,
            **kwargs
        )

        messages = [
            mock_position_message(pos_Lla.lat, pos_Lla.lon, pos_Lla.alt),
            {"type": "hover_timer", "data": 3.0},
            {"type": "hover_timer", "data": 5.5}
        ]

        for message in messages:
            briar_hover.message_queue.put(message)

        outcome = briar_hover.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_gimbal.return_value.start.assert_called_once_with()
        mock_yaw_trajectory.return_value.setpoint_driver.start.assert_called_once_with()
        for actual, expected in zip(mock_gimbal.return_value.stare_position, (0.0, 0.0, 17.163)):
            self.assertAlmostEqual(actual, expected, 3)

        self.assertEqual(
            mock_yaw_trajectory.return_value.setpoint_driver.lla,
            ellipsoid_to_amsl((pos_Lla.lat, pos_Lla.lon, pos_Lla.alt))
        )

    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "RepeatTimer")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "YawTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "Gimbal")
    def test_BriarHover_loop_until_succeeded_no_stare_position(
        self,
        mock_gimbal,
        mock_yaw_trajectory,
        mock_repeat_timer
    ):
        from dr_onboard_autonomy.states import BriarHover
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        mock_hover_time = 8.2
        mock_stare_position = None
        briar_hover = BriarHover(
            hover_time=mock_hover_time,
            stare_position=mock_stare_position,
            **kwargs
        )

        amsl_pos = (39.744704, -105.036727, 5287.15)
        position_message = mock_position_message(*amsl_to_ellipsoid(amsl_pos))
        hover_timer_message = {"type": "hover_timer", "data": 0.26334}

        briar_hover.message_queue.put(position_message)
        briar_hover.message_queue.put(hover_timer_message)

        outcome = briar_hover.execute(mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_yaw_trajectory.return_value.setpoint_driver.start.assert_called_once_with()
        mock_gimbal.assert_not_called()
        mock_gimbal.return_value.start.assert_not_called()