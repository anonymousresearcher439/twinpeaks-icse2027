import sys
import unittest
from unittest.mock import NonCallableMock, patch

from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    convert_LlaDict_to_tuple,
    convert_tuple_to_LlaDict
)
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from dr_onboard_autonomy.states.components.Setpoint import SetpointType
from dr_onboard_autonomy.models import ALTITUDE_REFERENCE, LlaPosition

from .mock_types import mock_copter_drone, mock_message_sender_position_message

class TestBriarHover(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)


    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "UpdateSpacialData")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "YawTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "Gimbal")
    def test_BriarHover_execute_loop_until_succeeded(
        self,
        mock_gimbal,
        mock_trajectory,
        mock_update_spacial_data
    ):
        from dr_onboard_autonomy.states import BriarHover

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {
                'setpoint_type': SetpointType.LLA,
                'setpoint_value': (39.744704, -105.036727, 5287.15)
            },
        }

        mock_userdata = NonCallableMock()
        pos_Lla = LlaPosition(39.744704, -105.036727, 5287.15, ALTITUDE_REFERENCE.AMSL)
        pos_Lla.to_wgs84_ellipsoid()
        stare_LlaDict = convert_tuple_to_LlaDict((39.747714, -105.048456, 5280.2481))
        briar_hover = BriarHover(
            hover_time=5.0,
            stare_position=stare_LlaDict,
            **kwargs
        )

        self.drone.data.location.add_position(1695270751.0, pos_Lla)
        messages = [
            mock_message_sender_position_message(pos_Lla.latitude, pos_Lla.longitude, pos_Lla.altitude),
            {"type": "hover_timer", "data": 3.0},
            {"type": "hover_timer", "data": 5.5}
        ]

        for message in messages:
            briar_hover.message_queue.put(message)

        outcome = briar_hover.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_gimbal.return_value.start.assert_called_once_with()
        mock_trajectory.return_value.start.assert_called_once_with()
        self.assertEqual(
            mock_gimbal.return_value.stare_position,
            amsl_to_ellipsoid(convert_LlaDict_to_tuple(stare_LlaDict))
        )
        setpoint_lla = mock_trajectory.return_value.setpoint_driver.lla
        previous_lla = kwargs['data']['setpoint_value']

        for actual, expected in zip (setpoint_lla, previous_lla):
            self.assertAlmostEqual(actual, expected)


    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "UpdateSpacialData")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BriarHover"], "HoldingTrajectory")
    def test_BriarHover_execute_loop_use_no_stare_position(
        self,
        mock_trajectory,
        mock_update_spacial_data
    ):
        from dr_onboard_autonomy.states import BriarHover

        pos_Lla = LlaPosition(39.744704, -105.036727, 5287.15, ALTITUDE_REFERENCE.AMSL)
        pos_Lla.to_wgs84_ellipsoid()
        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "data": {
                'setpoint_type': SetpointType.LLA,
                'setpoint_value': (39.744704, -105.036727, 5287.15)
            }
        }

        mock_userdata = NonCallableMock()

        briar_hover = BriarHover(
            hover_time=5.0,
            **kwargs
        )

        messages = [
            mock_message_sender_position_message(pos_Lla.latitude, pos_Lla.longitude, pos_Lla.altitude),
            {"type": "hover_timer", "data": 3.0},
            {"type": "hover_timer", "data": 5.5}
        ]

        for message in messages:
            briar_hover.message_queue.put(message)

        outcome = briar_hover.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_hover")
        mock_trajectory.return_value.start.assert_called_once_with()
