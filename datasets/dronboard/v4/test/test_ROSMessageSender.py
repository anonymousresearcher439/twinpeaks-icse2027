import unittest
from unittest.mock import (
    Mock,
    NonCallableMock,
    patch
)

from std_msgs.msg import Float64, String

from dr_onboard_autonomy.message_senders import ROSMessageSender

import mock_types


class TestStateMessageSender(unittest.TestCase):
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_init(self, rospy):
        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = Float64
        sender = ROSMessageSender(name, topic, topic_type)

        State = ROSMessageSender.State

        self.assertIsInstance(sender, ROSMessageSender)
        self.assertIsNone(sender.sub)
        self.assertIsNone(sender.to_state_machine)
        self.assertEqual(sender.current_state, State.STOPPED)

        self.assertEqual(sender.name, name)
        self.assertEqual(sender.topic, topic)
        self.assertEqual(sender.TopicType, topic_type)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_start_sub(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = Float64
        sender = ROSMessageSender(name, topic, topic_type)

        self.assertEqual(sender.current_state, State.STOPPED)
        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)
        self.assertIsNotNone(sender.sub)
        self.assertEqual(sender.sub, mock_subscriber)
        rospy.Subscriber.assert_called_once_with(topic, topic_type, sender.ros_callback)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_start(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        callback = Mock()
        sender.start(callback)
        self.assertEqual(sender.current_state, State.SENDING)
        self.assertEqual(sender.to_state_machine, callback)

    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_stop(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        callback = Mock()
        sender.start(callback)
        self.assertEqual(sender.current_state, State.SENDING)
        
        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_stop_subscriber(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)

        sender.stop_subscriber()
        mock_subscriber.unregister.assert_called_once()
        self.assertIsNone(sender.sub)
        self.assertEqual(sender.current_state, State.STOPPED)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_ros_callback_when_ready(self, rospy):
        """Test that we can receive a message when we're ready but not connected
        to a state machine.
        """
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)
        self.assertIsNone(sender.to_state_machine)

        test_message = String("test message")
        sender.ros_callback(test_message)
        self.assertTrue(True)

        sender.stop_subscriber()
        mock_subscriber.unregister.assert_called_once()
        self.assertIsNone(sender.sub)
        self.assertEqual(sender.current_state, State.STOPPED)

    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_ros_callback_when_sending(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        callback = Mock()
        self.assertIsNone(sender.to_state_machine)
        sender.start(callback)

        test_message = String("test message")
        sender.ros_callback(test_message)
        
        # make sure the call back was called once with a dict with the expected
        # keys and values
        callback.assert_called_once()
        args, kwargs = callback.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)

        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], name)
        self.assertIsInstance(message["data"], String)
        self.assertEqual(message["data"], test_message)

        sender.stop_subscriber()
        mock_subscriber.unregister.assert_called_once()
        self.assertIsNone(sender.sub)
        self.assertEqual(sender.current_state, State.STOPPED)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_start_and_stop(self, rospy):
        mock_subscriber = NonCallableMock(name="rospy.Subscriber")
        rospy.Subscriber.return_value = mock_subscriber
        State = ROSMessageSender.State

        name = "TEST_MESSAGE_SENDER"
        topic = "TEST_TOPIC"
        topic_type = String # the ROS string type
        sender = ROSMessageSender(name, topic, topic_type)
        self.assertEqual(sender.current_state, State.STOPPED)

        callback = Mock()
        self.assertIsNone(sender.to_state_machine)
        sender.start(callback)

        test_message = String("test message")
        sender.ros_callback(test_message)
        
        # make sure the call back was called once with a dict with the expected
        # keys and values
        callback.assert_called_once()
        args, kwargs = callback.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)

        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], name)
        self.assertIsInstance(message["data"], String)
        self.assertEqual(message["data"], test_message)

        sender.stop()
        self.assertIsNone(sender.to_state_machine)
        self.assertEqual(sender.current_state, State.READY)
        test_message2 = String("test message 2")
        sender.ros_callback(test_message2)

        callback2 = Mock()
        sender.start(callback2)
        # make sure rospy.Subscriber was still only called once!
        rospy.Subscriber.assert_called_once_with(topic, topic_type, sender.ros_callback)
        self.assertEqual(sender.current_state, State.SENDING)
        sender.ros_callback(test_message2)

        callback2.assert_called_once()
        args, kwargs = callback2.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)
        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], name)
        self.assertIsInstance(message["data"], String)
        self.assertEqual(message["data"], test_message2)

        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
        self.assertIsNone(sender.to_state_machine)

        sender.stop_subscriber()
        mock_subscriber.unregister.assert_called_once()
        self.assertIsNone(sender.sub)
        self.assertEqual(sender.current_state, State.STOPPED)