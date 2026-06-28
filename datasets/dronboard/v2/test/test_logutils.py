import sys
import unittest
from unittest.mock import (
    NonCallableMock,
    PropertyMock,
    patch
)

import rospy

from dr_onboard_autonomy import logutils


MOCK_TIME = 1681516811.8072336


def time_generator(start_time, delay_time):
    counter = 0
    while True:
        yield start_time + counter * delay_time + 0.000001
        counter += 1

def message_generator():
    counter = 0
    while True:
        yield f"Hello, world! {counter}"
        counter += 1


class TestAirLeaser(unittest.TestCase):

    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "rospy")
    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "time")
    def test_debouce_logger(self, mock_time, mock_rospy):
        delay_time = 1.0
        mock_time.time.side_effect = time_generator(MOCK_TIME, delay_time)
        
        # init DebounceLogger
        debouce_logger = logutils.DebounceLogger()

        str1 = "Hello, world!"
        str2 = "Hello, world again!"

        debouce_logger.info(str1, 3) # logged t=0
        debouce_logger.info(str2, 4) # logged t=1 (next log at t=5)

        debouce_logger.info(str1, 3) # skipped t=2
        debouce_logger.info(str2, 4) # skipped t=3

        debouce_logger.info(str1, 3) # logged t=4 (4 > 3) next log at t=7
        debouce_logger.info(str2, 4) # logged t=5 5 > 4 (next log at t=5+4=9)
        
        debouce_logger.info(str1, 3) # skipped t=6
        debouce_logger.info(str2, 4) # skipped t=7
        debouce_logger.info(str1, 3) # logged t=8 next log at t=11

        # should be called 5 times
        self.assertEqual(mock_rospy.loginfo.call_count, 5)
        call_args_list = mock_rospy.loginfo.call_args_list

        # should be called with str1 3 times
        call_args_list_str1 = [call_args for call_args in call_args_list if call_args[0][0] == str1]
        self.assertEqual(len(call_args_list_str1), 3)
        
        # should be called with str2 2 times
        call_args_list_str2 = [call_args for call_args in call_args_list if call_args[0][0] == str2]
        self.assertEqual(len(call_args_list_str2), 2)

    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "rospy")
    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "time")
    def test_debouce_logger_with_key_and_same_string(self, mock_time, mock_rospy):
        delay_time = 1.0
        mock_time.time.side_effect = time_generator(MOCK_TIME, delay_time)
        
        # init DebounceLogger
        debouce_logger = logutils.DebounceLogger()

        str1 = "Hello, world!"

        debouce_logger.info(str1, 3, key=1) # logged t=0 (next log at t=3)
        debouce_logger.info(str1, 3, key=2) # logged t=1 (next log at t=4)
        debouce_logger.info(str1, 3, key=1) # skipped t=2
        debouce_logger.info(str1, 3, key=2) # skipped t=3
        debouce_logger.info(str1, 3, key=1) # logged t=4 (4 > 3) next log at t=7
        debouce_logger.info(str1, 3, key=2) # logged t=5 5 > 4 (next log at t=5+4=9)
        debouce_logger.info(str1, 3, key=1) # skipped t=6

        # should be called 4 times
        self.assertEqual(mock_rospy.loginfo.call_count, 4)

        # We used the same string every time 
        call_args_list_key1 = [call_args for call_args in mock_rospy.loginfo.call_args_list if call_args[0][0] == str1]
        self.assertEqual(len(call_args_list_key1), 4)

    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "rospy")
    @patch.object(sys.modules["dr_onboard_autonomy.logutils"], "time")
    def test_debouce_logger_with_key_and_new_string_every_time(self, mock_time, mock_rospy):
        delay_time = 1.0
        mock_time.time.side_effect = time_generator(MOCK_TIME, delay_time)
        
        # init DebounceLogger
        debouce_logger = logutils.DebounceLogger()

        message_gen = message_generator()

        debouce_logger.info(next(message_gen), 3, key=1) # logged t=0 (next log at t=3) "Hello, world! 0"
        debouce_logger.info(next(message_gen), 3, key=2) # logged t=1 (next log at t=4) "Hello, world! 1"
        debouce_logger.info(next(message_gen), 3, key=1) # skipped t=2 "Hello, world! 2"
        debouce_logger.info(next(message_gen), 3, key=2) # skipped t=3 "Hello, world! 3"
        debouce_logger.info(next(message_gen), 3, key=1) # logged t=4 (4 > 3) next log at t=7 "Hello, world! 4"
        debouce_logger.info(next(message_gen), 3, key=2) # logged t=5 5 > 4 (next log at t=5+4=9) "Hello, world! 5"
        debouce_logger.info(next(message_gen), 3, key=1) # skipped t=6 "Hello, world! 6"

        # should be called 4 times
        self.assertEqual(mock_rospy.loginfo.call_count, 4)

        # We used a new string every time 
        # grab all the strings that were logged:
        call_args_list = [call_args[0][0] for call_args in mock_rospy.loginfo.call_args_list]

        # we should have 4 different strings that end in: 0, 1, 4, 5
        for i, actual in zip([0, 1, 4, 5], call_args_list):
            expected = f"Hello, world! {i}"
            self.assertEqual(actual, expected)
    