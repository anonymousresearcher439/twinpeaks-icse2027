import unittest

from threading import Event, Thread
from time import sleep
from typing import NewType, Tuple
from unittest.mock import Mock, NonCallableMock

import sensor_msgs.msg
import mavros_msgs.msg
import std_msgs.msg

from dr_onboard_autonomy.air_lease_service import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient

from .mock_types import mock_copter_drone, mock_message_sender_position_message, mock_imu_data


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
            air_lease_protocol_fsm=self.mock_air_lease_service,
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
        x=0.2705975
        y=0.2705985
        z=0.6532827
        w=0.6532803
        t=1719541089.5585942
        mock_imu_message = mock_imu_data(x=x, y=y, z=z, w=w, t=t)

        messages = [
            {"type": "imu", "data": mock_imu_message},
            {"type": "test_type", "data": "some position data a"},
            {"type": "test_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        # make sure the IMU data propigated into drone.data
        t_actual, quaternion = base_state.drone.data.attitude.last()
        self.assertEqual(t_actual, t)
        x_actual = quaternion[0]
        y_actual = quaternion[1]
        z_actual = quaternion[2]
        w_actual = quaternion[3]
        self.assertAlmostEqual(x_actual, x, 5)
        self.assertAlmostEqual(y_actual, y, 5)
        self.assertAlmostEqual(z_actual, z, 5)
        self.assertAlmostEqual(w_actual, w, 5)

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
        mock_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("testing_type", mock_handler)
        x=0.2705975
        y=0.2705985
        z=0.6532827
        w=0.6532803
        t=1719535778.8658388
        mock_imu_message = mock_imu_data(x, y, z, w, t)
        messages = [
            {"type": "imu", "data": mock_imu_message},
            {"type": "testing_type", "data": "some position data a"},
            {"type": "testing_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        t_actual, quaternion = base_state.drone.data.attitude.last()
        self.assertEqual(t_actual, t)
        x_actual = quaternion[0]
        y_actual = quaternion[1]
        z_actual = quaternion[2]
        w_actual = quaternion[3]
        self.assertAlmostEqual(x_actual, x, 5)
        self.assertAlmostEqual(y_actual, y, 5)
        self.assertAlmostEqual(z_actual, z, 5)
        self.assertAlmostEqual(w_actual, w, 5)

        # for call in update_data_imu_call_args:
        #     self.assertNotIn("gimbal_attitude", call)
        #     self.assertNotIn("gimbal_attitude", call[1].values())



    def test_execute_loop_until_outcome(self):
        from dr_onboard_autonomy.states import BaseState

        base_state = BaseState(
            air_lease_protocol_fsm=self.mock_air_lease_service,
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
        self.assertEqual(outcome, mock_message_outcome)

    def test_data_update_message(self):
        from dr_onboard_autonomy.states import BaseState
        import mock_types
        from dr_onboard_autonomy.models import FCUCopterMode, FCUStatus, FCULandedStatus, RelativeAltitude
        kwargs = mock_types.mock_state_kwargs(dict(
            air_lease_service=self.mock_air_lease_service,
            drone=self.mock_drone,
            reusable_message_senders=self.mock_reusable_msg_senders,
            mqtt_client=self.mock_mqtt,
            local_mqtt_client=self.mock_mqtt_local,
            outcomes=["succeeded", "error"]
        ))
        base_state = BaseState(**kwargs)

        mock_user_data = NonCallableMock()
        mock_data_handler = Mock()
        mock_message_outcome = "succeeded"
        mock_data_handler.side_effect = (None, mock_message_outcome)
        base_state.handlers.add_handler("test_type", mock_data_handler)

        t = 1719535778.8658388
        lat, lon, alt = 41.60667198246375, -86.35558129009804, 228.5
        mock_position_message = mock_types.mock_position(lat, lon, alt, t)

        voltage = 14
        current = 4.5
        level = 0.5
        mock_battery_message = mock_types.mock_battery_data(voltage, current, level)

        mock_state_message = mock_types.mock_message_sender_state_message(
            armed=True,
            connected=True,
            mode=FCUCopterMode.OFFBOARD,
            status=FCUStatus.ACTIVE,
            landed_status=FCULandedStatus.IN_AIR
        )
        mock_state = mock_state_message['data']

        mock_rel_alt = RelativeAltitude(256.23)

        messages = [
            {"type": "position", "data": mock_position_message},
            {"type": "relative_altitude", "data": mock_rel_alt},
            mock_state_message,
            {"type": "battery", "data": mock_battery_message},
            {"type": "test_type", "data": "some sensor data d"},
            {"type": "test_type", "data": "some sensor data e"},
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)

        self.assertEqual(base_state.drone.data.mode, mock_state.mode)

        # check battery voltage, current, and level
        self.assertEqual(base_state.drone.data.battery.voltage, voltage)
        self.assertEqual(base_state.drone.data.battery.current, current)
        self.assertEqual(base_state.drone.data.battery.level, level)
        
        # check position's time, (latutude, longitude, and altitude)
        t_actual, position_actual = base_state.drone.data.location.last()
        lat_actual, lon_actual, alt_actual = position_actual
        self.assertEqual(t_actual, t)
        self.assertEqual(lat_actual, lat)
        self.assertEqual(lon_actual, lon)
        self.assertEqual(alt_actual, alt)

    def test_drone_heading_and_attitude_are_updated_in_drone_data(self):
        from dr_onboard_autonomy.states import BaseState
        from dr_onboard_autonomy.gimbal import Gimbal
        import dr_onboard_autonomy.message_converters as mc
        import mock_types
        from dr_onboard_autonomy.models import Quaternion
        
        type(self.mock_drone).gimbal = NonCallableMock(spec=Gimbal)
        type(self.mock_drone).gimbal.attitude = Quaternion(0.0, 0.0, 0.7071081, 0.7071055)
        

        self.mock_drone.gimbal._attitude = Quaternion(0.0, 0.0, 0.7071081, 0.7071055)
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
        mock_imu_data = mock_types.mock_imu_data(x=0, y=0, z=0, w=1, t=1719541089.5585942)

        mock_compass_data = mock_types.mock_compass_hdg_data(90.0)
        mock_compass_data = mc.ros_compass_heading_adapter(mock_compass_data)
        messages = [
            {"type": "imu", "data": mock_imu_data},
            {"type": "compass_hdg", "data": mock_compass_data},
            {"type": "test_type", "data": "some position data a"},
            {"type": "test_type", "data": "some position data b"}
        ]

        for message in messages:
            base_state.message_queue.put(message)

        outcome = base_state.execute(userdata=mock_user_data)
        
        data = self.mock_drone.data
        # check that the drone heading was updated
        self.assertEqual(data.heading, 90.0)
        # check the drone attitude was updated
        actual_attitude = data.attitude.get_attitude()
        expected_attitude = Quaternion(0, 0, 0, 1)
        for component in ["x", "y", "z", "w"]:
            
            actual = getattr(actual_attitude, component)
            expected = getattr(expected_attitude, component)
            self.assertAlmostEqual(actual, expected)
        