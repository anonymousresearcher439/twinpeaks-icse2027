import json
import unittest
from typing import Any
from unittest.mock import NonCallableMock, Mock

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.briar_helpers import briarlla_from_lat_lon_alt
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import VisionTrigger, UpdateStarePosition, Gimbal
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator


def mock_message_senders():

    vision_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    vision_msg_sender.name = "vision"
    def find(name):
        if name == "vision":
            return vision_msg_sender
        else:
            msg_sender = NonCallableMock(spec=AbstractMessageSender)
            return msg_sender

    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.find.side_effect = find
    return reusable_message_senders


def mock_state():
    state = BaseState(
        # when "found" is a posible outcome, then we need to automatically add the VisionTrigger component.
        outcomes=["found", "success"],
        air_lease_service=NonCallableMock(spec=AirLeaseService),
        drone=NonCallableMock(spec=MAVROSDrone),
        heartbeat_handler=False,
        local_mqtt_client=NonCallableMock(spec=MQTTClient),
        mqtt_client=NonCallableMock(spec=MQTTClient),
        name="test_state",
        reusable_message_senders=mock_message_senders(),
        trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
    )
    return state

def mock_gimbal_driver():
    gimbal_driver = NonCallableMock(spec=Gimbal)
    return gimbal_driver


def mock_message(type: str, data: Any):
    return {
        "type": type,
        "data": data
    }

def mock_vision_message():
    payload = """
    {
        "x": 0.5,
        "y": 0.5,
        "confidence": 0.9,
        "timestamp": 1,
        "x_res": 416,
        "y_res": 416,
        "head_pose": "Forward"
    }
    """
    mqtt_message = NonCallableMock()
    mqtt_message.payload = payload.encode("utf-8")
    mqtt_message.topic = "vision"
    return mock_message("vision", mqtt_message)

class TestUpdateStarePosition(unittest.TestCase):
    def test_the_init_method(self):
        state = mock_state()
        self.assertIsNotNone(state.vision_trigger)
        self.assertEqual(type(state.vision_trigger), VisionTrigger)

    def test_vision_message(self):
        state = mock_state()
        vision_msg = mock_vision_message()
        state.message_queue.put(vision_msg)
        outcome = state.execute(None)
        self.assertEqual(outcome, "found")
    
    def test_message_sender_was_added(self):
        state = mock_state()
        state.reusable_message_senders.find.assert_called_with("vision")
        vision_msg_sender = state.reusable_message_senders.find("vision")
        self.assertIn(vision_msg_sender, state.message_senders)


