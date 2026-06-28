#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_flight_states"
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
from dr_onboard_autonomy.states import Hover, Land, Takeoff


class TestLand(unittest.TestCase):
    def test_Land_state_execute_loop_until_message_says_the_drone_is_on_ground(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"land.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_msg_senders,
            "uav_id": "uav_id",
        }

        userdata = NonCallableMock()

        land_state = Land(**args)

        extended_state_messages = [
            {"type": "extended_state", "data": NonCallableMock(landed_state=2)},
            {"type": "extended_state", "data": NonCallableMock(landed_state=4)},
            {"type": "extended_state", "data": NonCallableMock(landed_state=1)},
        ]

        for message in extended_state_messages:
            land_state.message_queue.put(message)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = land_state.execute(userdata)
        mock_msg_sender.start.assert_called_with(land_state.message_queue.put)
        mock_msg_sender.stop.assert_called()
        self.assertEqual(outcome, "succeeded_land")
        self.assertEqual(True, land_state.message_queue.empty())

    def test_Land_state_when_drone_fails_to_change_to_land_mode(self):
        drone = NonCallableMock(spec=MAVROSDrone)

        # make drone.land() return False
        # this should cause the outcome to be 'error'
        drone.configure_mock(**{"land.return_value": False})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id",
        }

        userdata = NonCallableMock()
        land_state = Land(**args)
        outcome = land_state.execute(userdata)
        self.assertEqual(outcome, "error")
        mock_msg_sender.start.assert_not_called()
        mock_msg_sender.stop.assert_not_called()


class TestTakeoff(unittest.TestCase):
    def test_Takeoff_state_execute_loop_until_message_says_the_drone_is_at_altitude(
        self,
    ):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"takeoff.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id",
        }

        userdata = NonCallableMock()

        takeoff_state = Takeoff(**args)

        ros_messages = [
            {"type": "extended_state", "data": NonCallableMock(landed_state=3)},
            {"type": "relative_altitude", "data": NonCallableMock(data=12.0)},
            {"type": "extended_state", "data": NonCallableMock(landed_state=2)},
        ]

        for message in ros_messages:
            takeoff_state.message_queue.put(message)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = takeoff_state.execute(userdata)
        mock_msg_sender.start.assert_called_with(takeoff_state.message_queue.put)
        mock_msg_sender.stop.assert_called()
        self.assertEqual(outcome, "succeeded_takeoff")
        self.assertEqual(True, takeoff_state.message_queue.empty())

    def test_Takeoff_state_overshot_the_altitude_by_more_than_the_threshold(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"takeoff.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_message_senders.configure_mock(
            **{"find.return_value": mock_msg_sender}
        )

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_message_senders,
            "uav_id": "uav_id",
        }

        userdata = NonCallableMock()

        takeoff_state = Takeoff(**args)

        ros_messages = [
            {"type": "extended_state", "data": NonCallableMock(landed_state=3)},
            {"type": "extended_state", "data": NonCallableMock(landed_state=2)},
            {"type": "relative_altitude", "data": NonCallableMock(data=50.0)},
        ]

        for message in ros_messages:
            takeoff_state.message_queue.put(message)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = takeoff_state.execute(userdata)
        mock_msg_sender.start.assert_called_with(takeoff_state.message_queue.put)
        mock_msg_sender.stop.assert_called()
        self.assertEqual(outcome, "succeeded_takeoff")
        self.assertEqual(True, takeoff_state.message_queue.empty())


class TestHover(unittest.TestCase):
    def test_Hover_state_run_until_complete(self):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"hover.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id",
            "time_limit": 0.5,
        }

        userdata = NonCallableMock()

        hover_state = Hover(**args)
        outcome = hover_state.execute(userdata)
        self.assertEqual(outcome, "succeeded_hover")


class FlightStatesTestSuite(TestLand, TestTakeoff, TestHover):
    pass


if __name__ == "__main__":
    import rostest

    rospy.init_node("test_flight_states", anonymous=True)
    rostest.rosrun(PKG, "test_flight_states", FlightStatesTestSuite)
