#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_flight_states"
import importlib
from dr_onboard_autonomy.mavros_layer import SetpointType
from dr_onboard_autonomy.states.components import Setpoint
import roslib

roslib.load_manifest(PKG)

from droneresponse_mathtools import Lla
import unittest
from unittest.mock import (
    NonCallableMock,
    patch
)

from dr_onboard_autonomy.air_lease import make_waypoint_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    BriarLla,
    convert_tuple_to_LlaDict
)
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.models import (
    CopterDrone,
    FCULandedStatus
)
from dr_onboard_autonomy.states import BaseState, Hover, Land, Rtl
from mock_types import (
    mock_copter_drone,
    mock_message_sender_position_message,
    mock_message_sender_state_message
)


# We need to patch the modules where some states are defined.
# But doing this is tricky.
#
# These classes are defined in a module that's named after the class. For example
# the Land class is defined in `src/dr_onboard_autonomy/states/Land.py`.
# However, the `src/dr_onboard_autonomy/states/__init__.py` file also defines a variable named `Land`,
# which shadows the `Land.py` module when we try to import it.
#
# This shadowing problem exists for a few of the states, including `Land`, `Hover`, and `Rtl`.
#
# Therefore, we can't use `@patch("dr_onboard_autonomy.states.Land")` to patch the module,
# as it would instead patch the `Land` variable defined in `__init__.py`.
#
# Similarly, `@patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "my_variable")` won't work,
# because `"dr_onboard_autonomy.states.Land"` is a *variable*, not a module, so it can't be used as a key in `sys.modules`.
#
# To correctly patch the `Land.py` module, we can use this trick:
# First need to get a reference to the module object. We can do that using:
Land_module = importlib.import_module(Land.__module__)
# This gives us a reference to the module where `Land` class is defined.
# Now we can use `@patch.object` to patch this module.
# For example: `@patch.object(Land_module, "some_variable")` will patch "some_variable" in the
# `Land.py` module.
# We need to do this for BaseState, Hover, and Rtl too
BaseState_module = importlib.import_module(BaseState.__module__)
Hover_module = importlib.import_module(Hover.__module__)
Rtl_module = importlib.import_module(Rtl.__module__)


class TestLand(unittest.TestCase):
    @patch.object(Land_module, "Queue")
    @patch.object(Land_module, "ReadMessagesAirborne")
    @patch.object(Land_module, "AirLeaser")
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    def test_Land_state_execute_loop_until_message_says_the_drone_is_on_ground(
        self,
        mock_heartbeat_status_handler,
        mock_air_leaser,
        mock_read_position_state,
        mock_queue
    ):  
        drone = mock_copter_drone()
        drone.data.location = NonCallableMock()
        drone.data.attitude = NonCallableMock()

        drone._set_fcu_mode.return_value = True

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        mock_read_position_state.return_value.execute.return_value = "succeeded"
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_air_leaser.return_value.execute.return_value = "succeeded"

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_msg_senders,
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()

        land_state = Land(**args)
        '''
        add in outcomes that will exist when passed to states within Land
        '''
        args["outcomes"] = ["succeeded_land", "error", "failsafe", "rtl"]

        state_messages = [
            mock_message_sender_state_message(landed_status=FCULandedStatus.IN_AIR),
            mock_message_sender_state_message(landed_status=FCULandedStatus.IN_AIR),
            mock_message_sender_position_message(
                lat=39.747714,
                lon=-105.048456,
                alt=5294.95
            ),
            mock_message_sender_state_message(landed_status=FCULandedStatus.ON_GROUND),
        ]


        for message in state_messages:
            land_state.message_queue.put(message)

        # When we run execute, we expect the state machine to read two messages and then return 'succeeded'
        outcome = land_state.execute(userdata)
        mock_msg_sender.start.assert_called_with(land_state.message_queue.put)
        mock_msg_sender.stop.assert_called()
        air_leaser_call_kwargs = mock_air_leaser.call_args.kwargs
        self.assertEqual(
            air_leaser_call_kwargs["tunnel_func"].__name__,
            make_waypoint_air_tunnel_func(end_pos=Lla(pos_tuple[0], pos_tuple[1], 0)).__name__
        )
        mock_air_leaser.return_value.execute.assert_called_once_with(userdata)
        mock_air_leaser.return_value.communicate_route_complete.assert_called_once_with(
            Lla(latitude=39.747714, longitude=-105.048456, altitude=5294.95)
        )
        self.assertEqual(outcome, "succeeded_land")
        # self.assertEqual(True, land_state.message_queue.empty())


    @patch.object(Land_module, "Queue")
    @patch.object(Land_module, "ReadMessagesAirborne")
    @patch.object(Land_module, "AirLeaser")
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    def test_Land_state_when_drone_fails_to_change_to_land_mode(
            self,
            mock_heartbeat_status_handler,
            mock_air_leaser,
            mock_read_position_state,
            mock_queue
        ):
        drone = mock_copter_drone()
        drone._set_fcu_mode.return_value = False

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        mock_read_position_state.return_value.execute.return_value = "succeeded"
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_air_leaser.return_value.execute.return_value = "succeeded"

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()
        land_state = Land(**args)
        outcome = land_state.execute(userdata)
        self.assertEqual(outcome, "error")
        mock_msg_sender.start.assert_not_called()
        mock_msg_sender.stop.assert_not_called()


    @patch.object(Land_module, "Queue")
    @patch.object(Land_module, "ReadMessagesAirborne")
    @patch.object(Land_module, "AirLeaser")
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    def test_Land_state_with_air_lease_failure(
        self,
        mock_heartbeat_status_handler,
        mock_air_leaser,
        mock_read_position_state,
        mock_queue
    ):
        drone= mock_copter_drone()

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        mock_read_position_state.return_value.execute.return_value = "succeeded"
        pos_tuple = amsl_to_ellipsoid((39.747714, -105.048456, 5280.2481))
        pos_briar_lla = BriarLla(convert_tuple_to_LlaDict(pos_tuple), is_amsl=False)
        mock_queue.return_value.get.return_value = pos_briar_lla

        mock_air_lease_failure = "some air lease failure"
        mock_air_leaser.return_value.execute.return_value = mock_air_lease_failure

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_msg_senders,
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()
        land_state = Land(**args)

        outcome = land_state.execute(userdata)
        self.assertEqual(outcome, mock_air_lease_failure)
        mock_msg_sender.start.assert_not_called()
        mock_msg_sender.stop.assert_not_called()


    @patch.object(Land_module, "ReadMessagesAirborne")
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    def test_Land_state_with_read_position_failure(
        self,
        mock_heartbeat_status_handler,
        mock_read_position_state
    ):
        drone = mock_copter_drone()

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_msg_senders,
            "uav_id": "uav_id"
        }

        mock_read_position_failure = "some read position failure"
        mock_read_position_state.return_value.execute.return_value = mock_read_position_failure

        userdata = NonCallableMock()
        land_state = Land(**args)

        outcome = land_state.execute(userdata)
        self.assertEqual(outcome, mock_read_position_failure)
        mock_msg_sender.start.assert_not_called()
        mock_msg_sender.stop.assert_not_called()


class TestHover(unittest.TestCase):
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    def test_Hover_state_run_until_complete(self, mock_heartbeat_status_handler):
        drone = mock_copter_drone()

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id",
            "time_limit": 0.5,
            "data": {
                Setpoint.TYPE_DATA: SetpointType.LLA,
                Setpoint.VALUE_DATA: (41.606679226520356, -86.35555788330228, 240.0),
                Setpoint.YAW_DATA: 0.0,
                
            }
        }

        userdata = NonCallableMock()

        hover_state = Hover(**args)
        outcome = hover_state.execute(userdata)
        self.assertEqual(outcome, "succeeded_hover")


class TestRtl(unittest.TestCase):
    @patch.object(BaseState_module, "HeartbeatStatusHandler")
    @patch.object(Rtl_module, "AbortHover")
    def test_rtl_state(self, mock_abort_hover, mock_heartbeat_status_handler):
        mock_abort_hover.return_value.execute.return_value = "abort_hover_outcome"
        drone = mock_copter_drone()

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()
        rtl_state = Rtl(**args)
        outcome = rtl_state.execute(userdata)
        self.assertEqual(outcome, "abort_hover_outcome")


class FlightStatesTestSuite(TestLand, TestHover, TestRtl):
    pass


if __name__ == "__main__":
    import rostest
    import rospy

    rospy.init_node("test_flight_states", anonymous=True)
    rostest.rosrun(PKG, "test_flight_states", FlightStatesTestSuite)
