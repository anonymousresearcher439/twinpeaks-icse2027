import unittest

from threading import Event, Thread
from time import sleep
from typing import NewType, Tuple
from unittest.mock import Mock, NonCallableMock

import sensor_msgs.msg
import mavros_msgs.msg
import std_msgs.msg

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient

from .mock_types import mock_copter_drone, mock_message_sender_position_message


class TestBaseState(unittest.TestCase):
    def setUp(self):
        self.mock_air_lease_service = NonCallableMock(spec=AirLeaseService)
        self.mock_drone = mock_copter_drone()

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.mock_reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.mock_reusable_msg_senders.configure_mock(**{"find.return_value": self.mock_msg_sender})

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)


    def test_execute_loop_until_message_outcome(self):
        from dr_onboard_autonomy.states import BaseState
        self.mock_drone.update_data = Mock()
        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_position_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_position_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("position2", mock_position_handler)

        position_messages = [
            {"type": "position2", "data": "some position data a"},
            {"type": "position2", "data": "some position data b"},
        ]

        for message in position_messages:
            base_state.message_queue.put(message)

        mock_on_entry = Mock()
        mock_on_entry.return_value = None
        base_state.on_entry = mock_on_entry

        mock_on_exit = Mock()
        mock_on_exit.return_value = None
        base_state.on_exit = mock_on_exit

        outcome = base_state.execute(userdata=mock_user_data)

        self.assertEqual(outcome, mock_message_outcome)

        # TODO: replace line below with assertion that mock_drone.data.state_name == expected
        # self.mock_drone.update_data.assert_called_with("state_name", base_state.name)

        mock_on_entry.assert_called_once_with(mock_user_data)
        mock_on_exit.assert_called_once_with(mock_message_outcome, mock_user_data)
        self.assertEqual(mock_position_handler.call_count, 2)
        self.mock_msg_sender.start.assert_called_once_with(base_state.message_queue.put)
        self.mock_msg_sender.stop.assert_called_once()


    def test_execute_loop_until_on_exit_outcome(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_position_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_position_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("position2", mock_position_handler)

        position_messages = [
            {"type": "position2", "data": "some position data a"},
            {"type": "position2", "data": "some position data b"},
        ]

        for message in position_messages:
            base_state.message_queue.put(message)

        mock_on_exit = Mock()
        mock_on_exit_outcome = "some exit outcome"
        mock_on_exit.return_value = mock_on_exit_outcome
        base_state.on_exit = mock_on_exit

        outcome = base_state.execute(userdata=mock_user_data)

        self.assertEqual(outcome, mock_on_exit_outcome)

    
    def test_execute_loop_until_execute2_outcome(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            air_lease_service=self.mock_air_lease_service,
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_sensor_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_sensor_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("sensor", mock_sensor_handler)

        position_messages = [
            mock_message_sender_position_message(37, -118, 20),
            mock_message_sender_position_message(37.1, -118, 20),
        ]

        done_event = Event()
        def keep_adding_sensor_messages():
            while not done_event.is_set():
                for message in position_messages:
                    base_state.message_queue.put(message)
                sleep(0.1)
        msg_adder = Thread(target=keep_adding_sensor_messages)
        msg_adder.start()

        mock_execute2 = Mock()
        mock_execute2_outcome = "some execute2 outcome"
        mock_execute2.return_value = mock_execute2_outcome
        base_state.execute2 = mock_execute2

        mock_on_exit = Mock()
        mock_on_exit.return_value = None
        base_state.on_exit = mock_on_exit

        outcome = base_state.execute(userdata=mock_user_data)
        done_event.set()
        msg_adder.join()
        self.assertEqual(outcome, mock_execute2_outcome)


    def test_execute_loop_until_execute2_on_exit_outcome(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_position_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_position_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("position2", mock_position_handler)

        position_messages = [
            {"type": "position2", "data": "some position data a"},
            {"type": "position2", "data": "some position data b"},
        ]

        for message in position_messages:
            base_state.message_queue.put(message)

        mock_execute2 = Mock()
        mock_execute2.return_value = None
        base_state.execute2 = mock_execute2

        mock_on_exit = Mock()
        mock_on_exit_outcome = "some exit outcome"
        mock_on_exit.return_value = mock_on_exit_outcome
        base_state.on_exit = mock_on_exit

        outcome = base_state.execute(userdata=mock_user_data)

        self.assertEqual(outcome, mock_on_exit_outcome)


    @unittest.skip("TODO - Michael M. to update drone.data instead in GH386")
    def test_execute_loop_imu_message(self):
        from dr_onboard_autonomy.states import BaseState
        from dr_onboard_autonomy.gimbal import Gimbal

        Quaternion = NewType('Quaternion', Tuple[float, float, float, float])
        
        type(self.mock_drone).gimbal = NonCallableMock(spec=Gimbal)
        type(self.mock_drone).gimbal.attitude = Quaternion((0.0, 0.0, 0.7071081, 0.7071055))
        
        self.mock_drone.update_all = Mock()
        self.mock_drone.gimbal._attitude = Quaternion((0.0, 0.0, 0.7071081, 0.7071055))
        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_handler_func = Mock()
        mock_message_outcome = "succeeded"
        mock_handler_func.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("test_type", mock_handler_func)
        mock_imu_message = sensor_msgs.msg.Imu()
        mock_imu_message.orientation.x = 0.2705975
        mock_imu_message.orientation.y = 0.2705985
        mock_imu_message.orientation.z = 0.6532827
        mock_imu_message.orientation.w = 0.6532803

        messages = [
            {"type": "imu", "data": mock_imu_message},
            {"type": "test_type", "data": "some position data a"},
            {"type": "test_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        # base_state.drone.update_all.call_args is a tuple where
        # the first element is a tuple of positional arguments and
        # the second element is a dictionary of keyword arguments.
        # We need to access the positional argument so we can inspect it.
        update_data_call_args = base_state.drone.update_all.call_args[0][0]
        self.assertIn("drone_attitude", update_data_call_args)
        self.assertIn("gimbal_attitude", update_data_call_args)
        self.assertAlmostEqual(update_data_call_args["drone_attitude"]["x"], 0.2705975)
        self.assertAlmostEqual(update_data_call_args["drone_attitude"]["y"], 0.2705985)
        self.assertAlmostEqual(update_data_call_args["drone_attitude"]["z"], 0.6532827)
        self.assertAlmostEqual(update_data_call_args["drone_attitude"]["w"], 0.6532803)
        
        self.assertAlmostEqual(update_data_call_args["gimbal_attitude"]["x"], 0.0, 5)
        self.assertAlmostEqual(update_data_call_args["gimbal_attitude"]["y"], 0.0, 5)
        self.assertAlmostEqual(update_data_call_args["gimbal_attitude"]["z"], 1.0, 5)
        self.assertAlmostEqual(update_data_call_args["gimbal_attitude"]["w"], 0.0, 5)


    def test_execute_loop_imu_message_gimbal_attitude_none(self):
        from dr_onboard_autonomy.states import BaseState
        from dr_onboard_autonomy.gimbal import Gimbal
        
        type(self.mock_drone).gimbal = NonCallableMock(spec=Gimbal)
        type(self.mock_drone).gimbal.attitude = None
        self.mock_drone.update_data = Mock()
        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_position_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_position_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("testing_type", mock_position_handler)
        mock_imu_message = sensor_msgs.msg.Imu()
        mock_imu_message.orientation.x = 0.2705975
        mock_imu_message.orientation.y = 0.2705985
        mock_imu_message.orientation.z = 0.6532827
        mock_imu_message.orientation.w = 0.6532803

        messages = [
            {"type": "imu", "data": mock_imu_message},
            {"type": "testing_type", "data": "some position data a"},
            {"type": "testing_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        update_data_imu_call_args = base_state.drone.update_data.call_args_list
        for call in update_data_imu_call_args:
            self.assertNotIn("gimbal_attitude", call)
            self.assertNotIn("gimbal_attitude", call[1].values())



    def test_execute_loop_airlease_cleanup(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            air_lease_service=self.mock_air_lease_service,
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_data_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_data_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("test_type", mock_data_handler)

        mock_position_message = mock_message_sender_position_message(
            lat=12.5,
            lon=-123.56,
            alt=236.2134
        )

        '''
        position message first to fill BaseState.message_data with position
        then put repeat timer message for airlease cleanup
        '''
        messages = [
            mock_position_message,
            {"type": "airlease_cleanup_timer", "data": 1.1},
            {"type": "test_type", "data": "some sensor data a"},
            {"type": "test_type", "data": "some sensor data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        self.mock_air_lease_service.send_cleanup.assert_called_once_with(
            position=(
                mock_position_message["data"].latitude,
                mock_position_message["data"].longitude,
                mock_position_message["data"].altitude
            )
        )


    @unittest.skip("TODO - Michael M. to update drone.data instead in GH386")
    def test_data_update_message(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            air_lease_service=self.mock_air_lease_service,
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_data_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_data_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("test_type", mock_data_handler)

        mock_position_message = sensor_msgs.msg.NavSatFix()
        mock_position_message.latitude
        mock_position_message.longitude
        '''
        Altitude above ellipsoid
        '''
        mock_position_message.altitude

        mock_battery_message = sensor_msgs.msg.BatteryState()
        mock_battery_message.voltage = 1.0
        mock_battery_message.current = 2.0
        mock_battery_message.percentage = 3.0

        mock_state = mavros_msgs.msg.State()
        mock_state.system_status = 25
        mock_state.mode = "some mode"
        mock_state.armed = True

        mock_rel_alt = std_msgs.msg.Float64({"rel_alt": 256.23})

        messages = [
            {"type": "position", "data": mock_position_message},
            {"type": "relative_altitude", "data": mock_rel_alt},
            {"type": "state", "data": mock_state},
            {"type": "battery", "data": mock_battery_message},
            {"type": "test_type", "data": "some sensor data d"},
            {"type": "test_type", "data": "some sensor data e"},
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        self.assertEqual(base_state.drone.get_data("mode"), mock_state.mode)

        battery_status = {
            "voltage": mock_battery_message.voltage,
            "current": mock_battery_message.current,
            "level": mock_battery_message.percentage,
        }
        self.assertEqual(base_state.drone.get_data("battery"), battery_status)

        position_status = {
            "latitude": mock_position_message.latitude,
            "longitude": mock_position_message.longitude,
            "altitude": mock_rel_alt.data,
        }
        self.assertEqual(base_state.drone.get_data("location"), position_status)


    @unittest.skip("TODO - Michael M. to update drone.data instead in GH386")
    def test_drone_heading_and_attitude_are_updated_in_drone_data(self):
        from dr_onboard_autonomy.states import BaseState
        from dr_onboard_autonomy.gimbal import Gimbal

        Quaternion = NewType('Quaternion', Tuple[float, float, float, float])
        
        type(self.mock_drone).gimbal = NonCallableMock(spec=Gimbal)
        type(self.mock_drone).gimbal.attitude = Quaternion((0.0, 0.0, 0.7071081, 0.7071055))
        

        self.mock_drone.gimbal._attitude = Quaternion((0.0, 0.0, 0.7071081, 0.7071055))
        base_state = BaseState(
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        )

        mock_user_data = NonCallableMock()
        mock_position_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_position_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("test_type", mock_position_handler)
        mock_imu_message = sensor_msgs.msg.Imu()
        mock_imu_message.orientation.x = 0.0
        mock_imu_message.orientation.y = 0.0
        mock_imu_message.orientation.z = 0.0
        mock_imu_message.orientation.w = 1.0

        messages = [
            {"type": "imu", "data": mock_imu_message},
            {"type": "compass_hdg", "data": std_msgs.msg.Float64(90.0)},
            {"type": "test_type", "data": "some position data a"},
            {"type": "test_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        
        
        data = self.mock_drone.data
        status_message = data.to_dict()
        print(status_message)
        attitude = status_message["status"]["drone_attitude"]
        expected_attitude = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        for component in ["x", "y", "z", "w"]:
            actual = attitude[component]
            expected_val = expected_attitude[component]
            self.assertAlmostEqual(actual, expected_val)

        heading = status_message["status"]["drone_heading"]
        self.assertAlmostEqual(heading, 90.0, 1)
