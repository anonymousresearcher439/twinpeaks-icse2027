#!/usr/bin/env python3

import json
import inspect
import unittest
from unittest.mock import (
    MagicMock,
    Mock,
    NonCallableMock,
    patch,
    PropertyMock
)
from threading import Event
from queue import Empty, Queue

from sensor_msgs.msg import BatteryState


from dr_onboard_autonomy.message_senders import ROSMessageSender

from dr_onboard_autonomy.models import (
    Battery,
    FCUCopterMode,
)


# get the module that contains the ROSMessageSender
message_sender_module = inspect.getmodule(ROSMessageSender)


class TestBatteryMessageSender(unittest.TestCase):
    def test_placeholder(self):
        # just pass the test
        self.assertTrue(True)

