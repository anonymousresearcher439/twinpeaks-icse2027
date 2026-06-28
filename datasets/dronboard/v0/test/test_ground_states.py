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
from dr_onboard_autonomy.states import Arm, Disarm, Preflight


def _create_args(drone, reusable_msg_senders):
    return {
        "drone": drone,
        "reusable_message_senders": reusable_msg_senders,
        "uav_id": "uav_id",
        "data": {},
        "mqtt_client": NonCallableMock()
    }
    

@unittest.skip("need to move this to an integration test")
class TestArm(unittest.TestCase):
    def test_Arm_state_execute_loop_until_state_message_says_the_drone_is_armed(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"arm.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = _create_args(drone, reusable_msg_senders)

        userdata = NonCallableMock()

        arm_state = Arm(**args)

        mock_state_message1 = {"type": "state", "data": NonCallableMock(armed=False)}
        mock_state_message2 = mock_state_message1.copy()
        mock_state_message2["data"] = NonCallableMock(armed=True)

        arm_state.message_queue.put(mock_state_message1)
        arm_state.message_queue.put(mock_state_message2)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = arm_state.execute(userdata)
        mock_msg_sender.start.assert_called_once_with(arm_state.message_queue.put)
        self.assertEqual(outcome, "succeeded")
        self.assertEqual(True, arm_state.message_queue.empty())

    def test_Arm_state_when_drone_fails_to_arm(self):
        drone = NonCallableMock(spec=MAVROSDrone)

        # make drone.arm() return False
        # this should cause the outcome to be 'error'
        drone.configure_mock(**{"arm.return_value": False})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = _create_args(drone, reusable_msg_senders)

        userdata = NonCallableMock()
        arm_state = Arm(**args)
        outcome = arm_state.execute(userdata)
        self.assertEqual(outcome, "error")

    def test_Arm_state_when_queue_gets_unknown_message(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"arm.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = _create_args(drone, reusable_msg_senders)

        arm_state = Arm(**args)

        mock_state_message = {"type": "state", "data": NonCallableMock(armed=False)}
        arm_state.message_queue.put(mock_state_message)

        mock_unknown_message = {"type": "???", "data": NonCallableMock(armed=True)}
        arm_state.message_queue.put(mock_unknown_message)

        mock_state_message_done = {"type": "state", "data": NonCallableMock(armed=True)}
        arm_state.message_queue.put(mock_state_message_done)

        userdata = NonCallableMock()
        outcome = arm_state.execute(userdata)

        self.assertEqual(outcome, "succeeded")
        self.assertEqual(True, arm_state.message_queue.empty())

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
