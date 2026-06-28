#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_message_senders"
import roslib
import rospy

roslib.load_manifest(PKG)

import unittest
from queue import Empty, Queue
from threading import Event, Thread

from std_msgs.msg import String

from dr_onboard_autonomy.message_senders import ROSMessageSender


class TestROSMessageSender(unittest.TestCase):
    def setUp(self) -> None:
        return super().setUp()

    def tearDown(self) -> None:
        return super().tearDown()

    def test_init_for_ROSMessageSender(self):
        e = ROSMessageSender(
            "test_init_ros_message_sender", "/birdy0/test_init", String
        )
        self.assertEqual(e.current_state, "stopped", "not constructed in stopped state")
        self.assertEqual(e.sub, None, "sub should be none when in the stopped state")
        self.assertEqual(e.name, "test_init_ros_message_sender", "name not set")
        self.assertEqual(e.topic, "/birdy0/test_init", "topic not set")
        self.assertEqual(e.TopicType, String, "TopicType not set")

    def test_start_method_for_ROSMessageSender(self):
        e = ROSMessageSender("test_sender", "/birdy0/test_start", String)
        e.start_subscriber()
        self.assertEqual(e.current_state, "started", "not constructed in stopped state")

    def test_ros_callback_ROSMessageSender(self):
        result_queue = Queue()

        ros_sender = ROSMessageSender(
            "test_ros_callback", "/birdy0/test_ros_callback", String
        )
        ros_sender.start(result_queue.put)

        ros_sender.ros_callback("test_data")

        expected_message = {"type": "test_ros_callback", "data": "test_data"}
        actual_message = result_queue.get(timeout=1)
        self.assertEqual(
            actual_message, expected_message, "didn't send the correct message"
        )

    def test_start_on_ROSMessageSender(self):
        connection_event = Event()

        class CommandSender(ROSMessageSender):
            def ros_callback(self, data):
                super().ros_callback(data.data)

        sender_name = "test_start_sending"
        topic = "/birdy0/commands"
        ros_sender = CommandSender(sender_name, topic, String)
        test_queue = Queue()
        ros_sender.start(test_queue.put)
        s = f"\n\n\n######\nname = {__name__}\n\n{2 + 1}"
        # rospy.logwarn(s)
        z = dir(ros_sender.sub)
        # rospy.logwarn(z)
        self.assertEqual(
            ros_sender.current_state, "sending", "not in the sending state"
        )

        def pub_message():
            # rospy.loginfo("pub_message thread started")
            pub = rospy.Publisher(topic, String, queue_size=10, latch=False)
            # rospy.loginfo("we made a Publisher")
            # rospy.loginfo(f"pub has this many connections: {pub.get_num_connections()}")
            # rospy.loginfo("waiting for a connection...")
            connection_event.wait()
            # rospy.loginfo(f"pub has this many connections: {pub.get_num_connections()}")
            pub.publish("test")
            # rospy.logwarn("pub.publish() done")

        t = Thread(target=pub_message)
        t.start()
        while ros_sender.sub.get_num_connections() < 1:
            rospy.sleep(0.1)
            # rospy.loginfo(f"sender has this many connections: {ros_sender.sub.get_num_connections()}")
        connection_event.set()
        t.join()

        expected_message = {"type": sender_name, "data": "test"}

        actual_message = test_queue.get(timeout=5)
        print(f"{actual_message=}", actual_message)
        self.assertEqual(
            actual_message, expected_message, "didn't send the correct message"
        )

        ros_sender.stop()
        self.assertEqual(ros_sender.current_state, "stopped")

    def test_stop_on_ROSMessageSender(self):
        sender_name = "test_stop"
        topic = "/birdy0/commands"
        ros_sender = ROSMessageSender(sender_name, topic, String)

        self.assertEqual(ros_sender.current_state, "stopped")

        def do_nothing_callback(message):
            pass

        ros_sender.start(do_nothing_callback)

        self.assertEqual(ros_sender.current_state, "sending")

        ros_sender.stop()
        self.assertEqual(ros_sender.current_state, "stopped")


def setUpModule():
    rospy.init_node("test_message_senders_node", anonymous=True)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_message_senders", TestROSMessageSender)
