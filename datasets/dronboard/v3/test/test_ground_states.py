#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_ground_states"
import roslib
import rospy

roslib.load_manifest(PKG)

import unittest
from unittest.mock import NonCallableMock

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import Arm, Disarm
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator


from .mock_types import (
    mock_copter_drone,
    mock_message,
    mock_message_sender_position_message,
    mock_message_sender_state_message
)


def _create_args(drone, reusable_msg_senders):
    return {
        "drone": drone,
        "reusable_message_senders": reusable_msg_senders,
        "uav_id": "uav_id",
        "data": {},
        "mqtt_client": NonCallableMock()
    }


class TestArm(unittest.TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_trajectory = NonCallableMock(spec=TrajectoryGenerator)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)

        self.kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local
        }


    def test_arm_successfully_in_on_entry_with_arm_position_received(self):
        mock_userdata = NonCallableMock()
        arm = Arm(**self.kwargs)

        mock_messages = (
            mock_message_sender_position_message(*(39.744704, -105.036727, 5287.15)),
            mock_message_sender_state_message()
        )

        for message in mock_messages:
            arm.message_queue.put(message)

        outcome = arm.execute(userdata=mock_userdata)
        self.assertEqual(outcome, "succeeded_armed")

        self.assertEqual(self.drone.data.arm_position.latitude, 39.744704)
        self.assertEqual(self.drone.data.arm_position.longitude, -105.036727)
        self.assertEqual(self.drone.data.arm_position.altitude, 5287.15)


    def test_armed_but_position_not_received_until_second_state_message(self):
        mock_userdata = NonCallableMock()
        arm = Arm(**self.kwargs)

        mock_messages = (
            mock_message_sender_state_message(),
            mock_message_sender_position_message(*(39.744704, -105.036727, 5287.15)),
            mock_message_sender_state_message()
        )

        for message in mock_messages:
            arm.message_queue.put(message)

        outcome = arm.execute(userdata=mock_userdata)
        self.assertEqual(outcome, "succeeded_armed")


    def test_multiple_attempts_to_arm(self):
        self.drone.arm.return_value = False
        mock_userdata = NonCallableMock()
        arm = Arm(**self.kwargs)

        mock_messages = (
            mock_message_sender_state_message(armed=False),
            mock_message_sender_position_message(*(39.744704, -105.036727, 5287.15)),
            mock_message("retry_arm", None),
            mock_message_sender_state_message(armed=False),
            mock_message("retry_arm", None),
            mock_message_sender_state_message(armed=True)
        )

        for message in mock_messages:
            arm.message_queue.put(message)

        outcome = arm.execute(userdata=mock_userdata)
        self.assertEqual(outcome, "succeeded_armed")
        self.assertEqual(self.drone.arm.call_count, 3)


@unittest.skip("need to move this to an integration test")
class TestDisarm(unittest.TestCase):
    def test_Disarm_state_with_one_state_message(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"arm.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = _create_args(drone, reusable_msg_senders)

        userdata = NonCallableMock()

        disarm_state = Disarm(**args)

        state_message = {"type": "state", "data": NonCallableMock(armed=False)}

        disarm_state.message_queue.put(state_message)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = disarm_state.execute(userdata)
        mock_msg_sender.start.assert_called_once_with(disarm_state.message_queue.put)
        self.assertEqual(outcome, "succeeded_disarm")
        self.assertEqual(True, disarm_state.message_queue.empty())


class GroundStatesTestSuite(TestArm, TestDisarm):
    pass


if __name__ == "__main__":
    import rostest

    rospy.init_node("test_ground_states", anonymous=True)
    rostest.rosrun(PKG, "test_ground_states", GroundStatesTestSuite)
