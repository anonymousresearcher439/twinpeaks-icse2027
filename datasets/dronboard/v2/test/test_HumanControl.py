import unittest
from unittest.mock import Mock, NonCallableMock

from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import HumanControl

from .mock_types import mock_copter_drone, mock_mqtt_client

class TestHumanControl(unittest.TestCase):
    def test_how_we_build_it(self):
        drone = mock_copter_drone()
        mqtt = NonCallableMock(spec=MQTTClient)
        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_message_senders.configure_mock(
            **{"find.return_value": mock_msg_sender}
        )

        args = {
            "drone": drone,
            "mqtt_client": mqtt,
            "reusable_message_senders": reusable_message_senders,
            "uav_id": "uav_id",
        }
        userdata = NonCallableMock()

        human_control = HumanControl(**args)

        messages = [
            {"type": "shutdown", "data": True},
        ]
        for m in messages:
            human_control.message_queue.put(m)

        outcome = human_control.execute(userdata)
        self.assertEqual(outcome, "error")


    def test_that_status_messages_are_sent_over_mqtt(self):
        drone = mock_copter_drone()
        drone.data = Mock()
        drone.gimbal = Mock()
        mqtt = mock_mqtt_client(local_ip="1.2.3.4")
        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_message_senders.configure_mock(
            **{"find.return_value": mock_msg_sender}
        )

        args = {
            "drone": drone,
            "mqtt_client": mqtt,
            "reusable_message_senders": reusable_message_senders,
            "uav_id": "uav_id",
        }
        userdata = NonCallableMock()

        human_control = HumanControl(**args)

        battery_data = NonCallableMock(voltage=15.21, current=9.86, percentage=0.55)
        state_data = NonCallableMock(
            system_status=255
        )  # Not sure why the front-end wants this and not mode?
        position = NonCallableMock(latitude=89.987, longitude=179.987)
        relative_altitude = NonCallableMock(data=10.0)

        # These messages should cause the state to build a message and send it
        messages = [
            {"type": "battery", "data": battery_data},
            {"type": "state", "data": state_data},
            {"type": "position", "data": position},
            {"type": "relative_altitude", "data": relative_altitude},
            {"type": "StatusMessage__send_status_timer", "data": 1.00001},
            {"type": "shutdown", "data": True},
        ]
        for m in messages:
            human_control.message_queue.put(m)

        outcome = human_control.execute(userdata)
        self.assertEqual(outcome, "error")
        self.assertEqual(mqtt.publish.call_args[0][0], "update_drone")
