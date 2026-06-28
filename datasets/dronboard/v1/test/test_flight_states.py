#!/usr/bin/env python3
PKG = "dr_onboard_autonomy"
NAME = "test_flight_states"
import roslib
import sys

roslib.load_manifest(PKG)

from std_msgs.msg import Header, Time
from sensor_msgs.msg import NavSatFix

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
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.states import Hover, Land, Rtl, UnstableTakeoff


class TestLand(unittest.TestCase):
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Land_state_execute_loop_until_message_says_the_drone_is_on_ground(
        self,
        mock_heartbeat_status_handler,
        mock_air_leaser,
        mock_read_position_state,
        mock_queue
    ):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.position_data = NonCallableMock()
        drone.attitude_data = NonCallableMock()

        drone.configure_mock(**{"land.return_value": True})

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
        args["outcomes"] = ["succeeded_land", "error", "human_control", "rtl"]

        mock_navsatfix_msg = NonCallableMock()
        mock_navsatfix_msg.latitude=39.747714
        mock_navsatfix_msg.longitude=-105.048456
        mock_navsatfix_msg.altitude=5294.95
        mock_navsatfix_msg.header = NonCallableMock()
        mock_navsatfix_msg.header.stamp = NonCallableMock()
        mock_navsatfix_msg.header.stamp.to_sec.side_effect = lambda : 1.0
        # mock_landing_position = Lla(latitude=39.747714, longitude=-105.048456, altitude=5294.95)
        mock_landing_position = mock_navsatfix_msg
        extended_state_messages = [
            {"type": "extended_state", "data": NonCallableMock(landed_state=2)},
            {"type": "extended_state", "data": NonCallableMock(landed_state=4)},
            {"type": "position", "data": mock_landing_position},
            {"type": "extended_state", "data": NonCallableMock(landed_state=1)},
        ]

        for message in extended_state_messages:
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
        mock_air_leaser.return_value.execute.assert_called_once_with(userdata=userdata)
        mock_air_leaser.return_value.communicate_route_complete.assert_called_once_with(
            Lla(latitude=39.747714, longitude=-105.048456, altitude=5294.95)
        )
        self.assertEqual(outcome, "succeeded_land")
        # self.assertEqual(True, land_state.message_queue.empty())


    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Land_state_when_drone_fails_to_change_to_land_mode(
            self,
            mock_heartbeat_status_handler,
            mock_air_leaser,
            mock_read_position_state,
            mock_queue
        ):
        drone = NonCallableMock(spec=MAVROSDrone)

        # make drone.land() return False
        # this should cause the outcome to be 'error'
        drone.configure_mock(**{"land.return_value": False})

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


    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "Queue")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "AirLeaser")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Land_state_with_air_lease_failure(
        self,
        mock_heartbeat_status_handler,
        mock_air_leaser,
        mock_read_position_state,
        mock_queue
    ):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"land.return_value": True})

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


    @patch.object(sys.modules["dr_onboard_autonomy.states.Land"], "ReadPosition")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Land_state_with_read_position_failure(
        self,
        mock_heartbeat_status_handler,
        mock_read_position_state
    ):
        drone = NonCallableMock(spec=MAVROSDrone)

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


class TestTakeoff(unittest.TestCase):
    @unittest.skip("This tests the old Takeoff state")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Takeoff_state_execute_loop_until_message_says_the_drone_is_at_altitude(
        self,
        mock_heartbeat_status_handler
    ):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"takeoff.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()

        takeoff_state = UnstableTakeoff(**args)

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

    @unittest.skip("This tests the old Takeoff state")
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Takeoff_state_overshot_the_altitude_by_more_than_the_threshold(
            self, 
            mock_heartbeat_status_handler
        ):        
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
            "uav_id": "uav_id"
        }

        userdata = NonCallableMock()

        takeoff_state = UnstableTakeoff(**args)

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
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    def test_Hover_state_run_until_complete(self, mock_heartbeat_status_handler):
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"hover.return_value": True})

        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        reusable_senders = NonCallableMock(spec=ReusableMessageSenders)
        reusable_senders.configure_mock(**{"find.return_value": mock_msg_sender})

        args = {
            "drone": drone,
            "reusable_message_senders": reusable_senders,
            "uav_id": "uav_id",
            "time_limit": 0.5
        }

        userdata = NonCallableMock()

        hover_state = Hover(**args)
        outcome = hover_state.execute(userdata)
        self.assertEqual(outcome, "succeeded_hover")


class TestRtl(unittest.TestCase):
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    @patch.object(sys.modules["dr_onboard_autonomy.states.Rtl"], "AbortHover")
    def test_rtl_state(self, mock_abort_hover, mock_heartbeat_status_handler):
        mock_abort_hover.return_value.execute.return_value = "abort_hover_outcome"
        drone = NonCallableMock(spec=MAVROSDrone)
        drone.configure_mock(**{"hover.return_value": True})

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


class FlightStatesTestSuite(TestLand, TestTakeoff, TestHover, TestRtl):
    pass


if __name__ == "__main__":
    import rostest

    rospy.init_node("test_flight_states", anonymous=True)
    rostest.rosrun(PKG, "test_flight_states", FlightStatesTestSuite)
