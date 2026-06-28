#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_preflight"
import roslib
import rospy

roslib.load_manifest(PKG)

import unittest
from queue import Empty, Queue
from threading import Event, Thread

from std_msgs.msg import String

from dr_onboard_autonomy.states.Preflight import Preflight
from dr_onboard_autonomy.mavros_layer import MAVROSDrone


class TestPreflight(unittest.TestCase):
    def setUp(self):
        rospy
        rospy.wait_for_service("mavros/param/set", 30.0)


class TestPreflight(unittest.TestCase):
    def test_Preflight_all_conditions(self):
        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": None,
            "reusable_message_senders": reusable_msg_senders,
            "uav_id": "uav_id",
        }

        userdata = NonCallableMock()

        preflight_state = Preflight(**args)

        state_message = {"type": "state", "data": NonCallableMock(armed=False)}
        extended_state_message = {
            "type": "extended_state",
            "data": NonCallableMock(landed_state=1),
        }
        battery_message = {"type": "battery", "data": NonCallableMock(percentage=1.0)}
        position_message = {
            "type": "position",
            "data": NonCallableMock(status=NonCallableMock(status=1)),
        }
        for message in [
            state_message,
            extended_state_message,
            battery_message,
            position_message,
        ]:
            preflight_state.message_queue.put(message)

        outcome = preflight_state.execute(userdata)
        self.assertEqual(outcome, "succeeded")
        self.assertEqual(True, preflight_state.message_queue.empty())


def setUpModule():
    rospy.init_node("test_preflight_node", anonymous=True)
    uav_id = rospy.get_param("uav_id", default="birdy0")
    MAVROSDrone(uav_id)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_preflight", TestPreflight)
