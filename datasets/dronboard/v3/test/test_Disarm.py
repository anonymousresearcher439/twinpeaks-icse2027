import sys
import unittest
from unittest.mock import (
    NonCallableMock,
    patch
)

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from mock_types import mock_message_sender_state_message, mock_drone, mock_copter_drone, mock_shutdown_message


class TestDisarm(unittest.TestCase):
    def setUp(self):
        # self.drone = NonCallableMock(spec=MAVROSDrone)
        
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)
    
    
    @patch.object(sys.modules["dr_onboard_autonomy.states.Disarm"], "AirLeaser")
    def test_Disarm(self, mock_air_leaser):
        from dr_onboard_autonomy.states import Disarm
        
        mock_air_leaser.return_value.land.return_value = "succeeded"
        mock_userdata = NonCallableMock()

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local

        }

        extended_state_messages = [
            mock_message_sender_state_message(armed=True),
            mock_message_sender_state_message(armed=False),
            mock_shutdown_message(),
        ]

        disarm = Disarm(**kwargs)

        kwargs["outcomes"] = ["succeeded_disarm", "error"]

        for message in extended_state_messages:
            disarm.message_queue.put(message)
        
        outcome = disarm.execute(userdata=mock_userdata)

        self.assertEqual(outcome, "succeeded_disarm")

        self.assertEqual(outcome, "succeeded_disarm")
        mock_air_leaser.assert_called_once_with(
            hold_position=False,
            **kwargs
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.Disarm"], "AirLeaser")
    def test_Disarm_air_lease_failure(self, mock_air_leaser):
        from dr_onboard_autonomy.states import Disarm
        
        mock_air_lease_outcome = "some failure"
        mock_air_leaser.return_value.land.return_value = mock_air_lease_outcome
        mock_userdata = NonCallableMock()

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local

        }

        disarm = Disarm(**kwargs)
        
        outcome = disarm.execute(userdata=mock_userdata)

        self.assertEqual(outcome, mock_air_lease_outcome)
