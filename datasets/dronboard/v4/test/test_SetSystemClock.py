import importlib
import json
import sys
import time
import unittest

from unittest.mock import patch, NonCallableMock, Mock

from dr_onboard_autonomy.states import SetSystemClock, BaseState
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from dr_onboard_autonomy.models import (
    GpsPositionMessage,
    DATUM_REFERENCE,
    TimeSource,
    TimeRef,
)

from . import mock_types


SetSystemClock_module = importlib.import_module(SetSystemClock.__module__)
BaseState_module = importlib.import_module(BaseState.__module__)



class TestSetSystemClock(unittest.TestCase):
    def test_init(self):
        args = mock_types.mock_state_kwargs()
        with patch.object(BaseState_module.smach.State, "__init__", return_value=None) as mock_smach_state:
            state = SetSystemClock(**args)
            # get the kwargs passed to the mock_smach_state.__init__ call
            mock_smach_state_kwargs = mock_smach_state.call_args[1]
            actual_outcomes = mock_smach_state_kwargs.get("outcomes")
            # make sure the outcomes are as expected
            self.assertSetEqual(set(actual_outcomes), {"succeeded", "skipped", "error"})
            
        self.assertIsInstance(state, SetSystemClock_module.SetSystemClock)
        # make sure we asked for the "time_reference" message sender
        # note we will have called the find method on the reusable_message_senders many times
        # but in this test we want to make sure one of those calls was for "time_reference"
        args["reusable_message_senders"].find.assert_any_call("time_reference")
        # also make sure we asked for the "position" message sender
        args["reusable_message_senders"].find.assert_any_call("position")

    def test_on_time_ref(self):
        args = mock_types.mock_state_kwargs()
        state = SetSystemClock(**args)

        nano_seconds = 1724954625977080097
        # we need a mock GpsPositionMessage to pass to the on_position method
        gps_pos = GpsPositionMessage(11, 22, 33, DATUM_REFERENCE.ELLIPSOID_WGS84, nano_seconds, True)
        message = {"data": gps_pos}
        state.on_position(message)

        self.assertTrue(state._is_gps_fix)

        time_ref = TimeRef(
            nano_seconds,
            TimeSource.FCU,
        )
        time_ref = NonCallableMock(wraps=time_ref)
        time_ref.nanoseconds.return_value = nano_seconds
        # since we have wrapped this we can override the adjusted_timestamp_ns method so it returns nano_seconds + 1000
        time_ref.adjusted_timestamp_ns.return_value = nano_seconds + 1000
        message = {"data": time_ref}
        
        # need to patch SetSystemClock_module.time so we don't change the system clock for real...
        with patch.object(SetSystemClock_module.time, "clock_settime_ns") as mock_clock_settime_ns:
            self.assertEqual(state.on_time_ref(message), "succeeded")
            mock_clock_settime_ns.assert_called_once_with(SetSystemClock_module.time.CLOCK_REALTIME, nano_seconds + 1000)
