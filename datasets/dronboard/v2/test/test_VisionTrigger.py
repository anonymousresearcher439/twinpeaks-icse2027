import json
from typing import Any, List
import unittest
from unittest.mock import NonCallableMock

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import VisionTrigger, Gimbal
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from test.mock_types import mock_copter_drone


def mock_message_senders():
    vision_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    vision_msg_sender.name = "vision"

    target_position_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    target_position_msg_sender.name = "target_position"

    def find(name):
        if name == "vision":
            return vision_msg_sender
        if name == "target_position":
            return target_position_msg_sender
        else:
            msg_sender = NonCallableMock(spec=AbstractMessageSender)
            return msg_sender

    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.find.side_effect = find
    return reusable_message_senders


def mock_state(outcomes_to_add: List[str]=[]):
    outcomes = ["success"]
    outcomes.extend(outcomes_to_add)

    state = BaseState(
        outcomes=outcomes,
        air_lease_service=NonCallableMock(spec=AirLeaseService),
        drone=mock_copter_drone(),
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
        "ts": 1,
        "x_res": 416,
        "y_res": 416,
    }
    """
    mqtt_message = NonCallableMock()
    mqtt_message.payload = payload.encode("utf-8")
    mqtt_message.topic = "vision"
    return mock_message("vision", mqtt_message)

def mock_target_position_message():
    payload = """
    {
        "latitude": 39.747394,
        "longitude": -105.044845,
        "altitude_amsl": 1578.56
    }
    """
    mqtt_message = NonCallableMock()
    mqtt_message.payload = payload.encode("utf-8")
    mqtt_message.topic = "target_position"
    return mock_message("target_position", mqtt_message)


class TestVisionTrigger(unittest.TestCase):
    def test_the_init_method(self):
        state = mock_state(outcomes_to_add=["found"])
        self.assertIsNotNone(state.vision_trigger)
        self.assertEqual(type(state.vision_trigger), VisionTrigger)

    def test_the_init_method_no_found_outcome(self):
        state = mock_state()
        with self.assertRaises(AttributeError):
            state.vision_trigger

    def test_vision_message(self):
        state = mock_state(outcomes_to_add=["found"])
        vision_msg = mock_vision_message()
        state.message_queue.put(vision_msg)
        outcome = state.execute(None)
        self.assertEqual(outcome, "found")

    def test_message_sender_was_added(self):
        state = mock_state(outcomes_to_add=["found"])
        state.reusable_message_senders.find.assert_called_with("vision")
        vision_msg_sender = state.reusable_message_senders.find("vision")
        self.assertIn(vision_msg_sender, state.message_senders)


class TestTargetPosition(unittest.TestCase):
    def test_TargetPosition_returns_target_position_outcome(self):
        state = mock_state(outcomes_to_add=["target_position"])

        state.message_queue.put(mock_target_position_message())
        outcome = state.execute(None)
        self.assertEqual(outcome, "target_position")

        target_position_saved = state.drone.data.target_position

        self.assertIsNotNone(target_position_saved)
        mock_target_pos_dict = json.loads(mock_target_position_message()["data"].payload)
        self.assertAlmostEqual(
            target_position_saved.latitude,
            mock_target_pos_dict["latitude"]
        )
        self.assertAlmostEqual(
            target_position_saved.longitude,
            mock_target_pos_dict["longitude"]
        )
        self.assertAlmostEqual(
            target_position_saved.altitude,
            mock_target_pos_dict["altitude_amsl"]
        )


    def test_message_sender_was_added(self):
        """Since reusable_message_senders is mocked, this test checks the TargetPosition component
        successfully adds the mocked target_position message sender to the state's message_senders
        """
        state = mock_state(outcomes_to_add=["target_position"])
        state.reusable_message_senders.find.assert_called_with("target_position")
        self.assertIn(
            state.reusable_message_senders.find("target_position"),
            state.message_senders
        )


