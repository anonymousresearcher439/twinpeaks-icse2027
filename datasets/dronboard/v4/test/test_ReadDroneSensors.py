import unittest
from unittest.mock import Mock, NonCallableMock

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient

from .mock_types import mock_copter_drone, mock_message_sender_position_message


class TestReadPosition(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)


    def test_process_on_position_message_from_message_queue(self):
        from dr_onboard_autonomy.states.ReadDroneSensors import ReadPosition

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_userdata = NonCallableMock()
        mock_return_channel = Mock()
        read_position = ReadPosition(return_channel=mock_return_channel, **kwargs)

        read_position.message_queue.put(mock_message_sender_position_message(
            *(39.744704, -105.036727, 5287.15)
        ))

        outcome = read_position.execute(userdata=mock_userdata)
        self.assertEqual(outcome, "succeeded")

        briar_lla_pos: BriarLla = mock_return_channel.put.call_args.args[0]

        self.assertEqual(briar_lla_pos.ellipsoid.tup, (39.744704, -105.036727, 5287.15))


    def test_on_position_message_no_gps_fix(self):
        from dr_onboard_autonomy.states.ReadDroneSensors import ReadPosition

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }

        mock_return_channel = Mock()
        read_position = ReadPosition(return_channel=mock_return_channel, **kwargs)

        outcome = read_position.on_position(
            message=mock_message_sender_position_message(
            *(39.744704, -105.036727, 5287.15),
            gps_fix=False
        ))

        self.assertIsNone(outcome)


