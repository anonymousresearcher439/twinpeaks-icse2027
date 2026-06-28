
import unittest
from unittest.mock import (
    Mock,
    NonCallableMock,
    patch
)
from mavros_msgs.msg import State as RosState
from mavros_msgs.msg import ExtendedState

from dr_onboard_autonomy.message_senders import StateMessageSender
from dr_onboard_autonomy.models import (
    FCUCopterMode,
    FCULandedStatus,
    FCUState,
    FCUStatus,
)

import mock_types


class TestStateMessageSender(unittest.TestCase):
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_init(self, rospy):
        sender = StateMessageSender()

        State = StateMessageSender.State

        self.assertIsInstance(sender, StateMessageSender)
        self.assertEqual(sender.current_state, State.STOPPED)
        self.assertIsNone(sender.to_state_machine)
        self.assertIsNone(sender.last_state)
        self.assertIsNone(sender.last_extended)

    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_start_sub(self, rospy):
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)
        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)

        state_topic = "mavros/state"
        state_type = RosState
        state_callback = sender.ros_callback_state
        rospy.Subscriber.assert_any_call(state_topic, state_type, state_callback)

        ext_state_topic = "mavros/extended_state"
        ext_state_type = ExtendedState
        ext_state_callback = sender.ros_callback_extended_state
        rospy.Subscriber.assert_any_call(ext_state_topic, ext_state_type, ext_state_callback)

        # Ensure rospy.Subscriber was called exactly twice
        self.assertEqual(rospy.Subscriber.call_count, 2)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_stop_sub(self, rospy):
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)

        sub1, sub2 = NonCallableMock(), NonCallableMock()
        rospy.Subscriber.side_effect = [sub1, sub2]

        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)

        state_sub, ext_state_sub = sender.state_sub, sender.extended_sub
        self.assertNotEqual(state_sub, ext_state_sub)

        self.assertIn(state_sub, [sub1, sub2])
        self.assertIn(ext_state_sub, [sub1, sub2])


        state_topic = "mavros/state"
        state_type = RosState
        state_callback = sender.ros_callback_state
        rospy.Subscriber.assert_any_call(state_topic, state_type, state_callback)

        ext_state_topic = "mavros/extended_state"
        ext_state_type = ExtendedState
        ext_state_callback = sender.ros_callback_extended_state
        rospy.Subscriber.assert_any_call(ext_state_topic, ext_state_type, ext_state_callback)

        # Ensure rospy.Subscriber was called exactly twice
        self.assertEqual(rospy.Subscriber.call_count, 2)

        sender.stop_subscriber()
        self.assertEqual(sender.current_state, State.STOPPED)
        self.assertIsNone(sender.state_sub)
        self.assertIsNone(sender.extended_sub)
        # make sure sub1.unregister() and sub2.unregister() were called
        sub1.unregister.assert_called_once()
        sub2.unregister.assert_called_once()

        sender.stop_subscriber()
        self.assertEqual(sender.current_state, State.STOPPED)
        # should still have only called unregister() once
        sub1.unregister.assert_called_once()
        sub2.unregister.assert_called_once()

    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_internal_state(self, rospy):        
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)

        to_state_machine = Mock()
        sender.start(to_state_machine)
        self.assertEqual(sender.current_state, State.SENDING)
        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
        sender.stop_subscriber()
        self.assertEqual(sender.current_state, State.STOPPED)

        sender.start_subscriber()
        self.assertEqual(sender.current_state, State.READY)
        sender.stop()
        self.assertEqual(sender.current_state, State.READY)

        sender.stop_subscriber()
        self.assertEqual(sender.current_state, State.STOPPED)
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_ros_callbacks(self, rospy):
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)

        to_state_machine = Mock()

        sender.start(to_state_machine)
        self.assertEqual(sender.current_state, State.SENDING)

        # make sure rospy.Subscriber was called twice
        self.assertEqual(rospy.Subscriber.call_count, 2) 

        state_data = mock_types.mock_state_message(
            mode="OFFBOARD",
            armed=True,
            connected=True,
            system_status=4, #  MAV_STATE_ACTIVE = 4	
        )["data"]

        sender.ros_callback_state(state_data)
        # make sure to_state_machine was called once
        to_state_machine.assert_called_once()
        args, kwargs = to_state_machine.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)
        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], "state")
        self.assertIsInstance(message["data"], FCUState)

        # make sure the FCUState object has the expected values
        fcu_state = message["data"]
        self.assertEqual(fcu_state.mode, FCUCopterMode.OFFBOARD)
        self.assertEqual(fcu_state.armed, True)
        self.assertEqual(fcu_state.connected, True)
        self.assertEqual(fcu_state.status, FCUStatus.ACTIVE)
        self.assertEqual(fcu_state.landed_status, FCULandedStatus.UNKNOWN)

        # reset to_state_machine 
        to_state_machine.reset_mock()

        # send an extended state message
        ext_state_data = mock_types.mock_ROS_ExtendedState(1) # set landed state to LANDED_STATE_ON_GROUND
        assert ext_state_data.landed_state == 1
        sender.ros_callback_extended_state(ext_state_data)

        # make sure to_state_machine was called once
        to_state_machine.assert_called_once()
        # Make sure we got called with the expected message
        args, kwargs = to_state_machine.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)
        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], "state")
        self.assertIsInstance(message["data"], FCUState)

        # make sure the FCUState object has the expected values
        fcu_state = message["data"]
        self.assertEqual(fcu_state.mode, FCUCopterMode.OFFBOARD)
        self.assertEqual(fcu_state.armed, True)
        self.assertEqual(fcu_state.connected, True)
        self.assertEqual(fcu_state.status, FCUStatus.ACTIVE)
        self.assertEqual(fcu_state.landed_status, FCULandedStatus.ON_GROUND)

        # reset to_state_machine again
        to_state_machine.reset_mock()

        # send another state message
        state_data2 = mock_types.mock_state_message(
            mode="POSITION", # an ardupilot mode
            armed=False,
            connected=True,
            system_status=4, #  MAV_STATE_ACTIVE = 4	
        )["data"]
        sender.ros_callback_state(state_data2)
        # make sure to_state_machine was called once
        to_state_machine.assert_called_once()
        args, kwargs = to_state_machine.call_args
        self.assertEqual(len(args), 1)
        self.assertEqual(len(kwargs), 0)
        message = args[0]
        self.assertIsInstance(message, dict)
        self.assertIn("type", message)
        self.assertIn("data", message)
        self.assertEqual(message["type"], "state")
        self.assertIsInstance(message["data"], FCUState)

        # make sure the FCUState object has the expected values
        fcu_state = message["data"]
        self.assertEqual(fcu_state.mode, FCUCopterMode.POS_HOLD)
        self.assertEqual(fcu_state.armed, False)
        self.assertEqual(fcu_state.connected, True)
        self.assertEqual(fcu_state.status, FCUStatus.ACTIVE)
        self.assertEqual(fcu_state.landed_status, FCULandedStatus.ON_GROUND)

        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
        self.assertIsNone(sender.to_state_machine)

        state_data3 = mock_types.mock_state_message(
            mode="GUIDED", # an ardupilot mode
            armed=True,
            connected=True,
            system_status=4, #  MAV_STATE_ACTIVE = 4	
        )["data"]
        sender.ros_callback_state(state_data3)
        self.assertEqual(sender.last_state, state_data3)
        self.assertEqual(sender.last_extended, ext_state_data)

        ext_state_data2 = mock_types.mock_ROS_ExtendedState(2) # set landed state to LANDED_STATE_IN_AIR
        sender.ros_callback_extended_state(ext_state_data2)
        self.assertEqual(sender.last_state, state_data3)
        self.assertEqual(sender.last_extended, ext_state_data2)
        self.assertIs(sender.last_extended, ext_state_data2)

        to_state_machine.reset_mock()
        sender.start(to_state_machine)
        self.assertEqual(sender.current_state, State.SENDING)
        sender.ros_callback_extended_state(ext_state_data2)
        to_state_machine.assert_called_once()
        args, kwargs = to_state_machine.call_args
        message = args[0]
        fcu_state = message["data"]
        self.assertEqual(fcu_state.mode, FCUCopterMode.OFFBOARD)
        self.assertEqual(fcu_state.landed_status, FCULandedStatus.IN_AIR)

        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
        sender.stop_subscriber()
        self.assertEqual(sender.current_state, State.STOPPED)

    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_ros_callbacks_extended_state_before_state1(self, rospy):
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)

        to_state_machine = Mock()

        sender.start(to_state_machine)
        self.assertEqual(sender.current_state, State.SENDING)

        ext_state_data = mock_types.mock_ROS_ExtendedState() # LANDED_STATE_IN_AIR
        sender.ros_callback_extended_state(ext_state_data)
        # make sure we didn't call to_state_machine
        to_state_machine.assert_not_called()

        state_data = mock_types.mock_state_message(
            mode="OFFBOARD",
            armed=True,
            connected=True,
            system_status=4, #  MAV_STATE_ACTIVE = 4	
        )["data"]
        sender.ros_callback_state(state_data)
        # make sure we called to_state_machine
        to_state_machine.assert_called_once()
    
    @patch('dr_onboard_autonomy.message_senders.rospy')
    def test_ros_callbacks_extended_state_before_state2(self, rospy):
        State = StateMessageSender.State

        sender = StateMessageSender()
        self.assertEqual(sender.current_state, State.STOPPED)

        to_state_machine = Mock()

        sender.start(to_state_machine)
        self.assertEqual(sender.current_state, State.SENDING)
        sender.stop()
        self.assertEqual(sender.current_state, State.READY)
        self.assertIsNone(sender.to_state_machine)

        ext_state_data = mock_types.mock_ROS_ExtendedState() # LANDED_STATE_IN_AIR
        sender.ros_callback_extended_state(ext_state_data)
        # make sure we didn't call to_state_machine
        to_state_machine.assert_not_called()

        state_data = mock_types.mock_state_message(
            mode="OFFBOARD",
            armed=True,
            connected=True,
            system_status=4, #  MAV_STATE_ACTIVE = 4	
        )["data"]
        sender.ros_callback_state(state_data)
        sender.start(to_state_machine)
        # make sure we didn't call to_state_machine
        to_state_machine.assert_not_called()
        sender.ros_callback_extended_state(ext_state_data)
        to_state_machine.assert_called_once()
        to_state_machine.reset_mock()

        sender.ros_callback_state(state_data)
        to_state_machine.assert_called_once()
