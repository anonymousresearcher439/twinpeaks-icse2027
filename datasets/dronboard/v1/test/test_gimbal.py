#!/usr/bin/env python3

import math
from typing import Tuple
import unittest
from unittest.mock import NonCallableMock, PropertyMock, patch
from threading import Event
from queue import Empty, Queue
from dr_onboard_autonomy.gimbal.gimbal import MavlinkNode
from dr_onboard_autonomy.mavlink import MavlinkSender
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator

from sensor_msgs.msg import Imu, NavSatFix
import tf

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.gimbal import Gimbal as GimbalController
from dr_onboard_autonomy.drone_data import Data
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders  ,
)
from dr_onboard_autonomy import gimbal_math
from dr_onboard_autonomy.gimbal.data_types import Quaternion
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.collections import MessageHandler
from dr_onboard_autonomy.states.components import Gimbal

def _create_reusable_message_senders():
    message_sender_names = [
        "position",
        "velocity",
        "relative_altitude",
        "battery",
        "state",
        "extended_state",
        "diagnostics",
        "estimator_status",
        "imu",
        "compass_hdg",
    ]
    msg_senders = {}
    for name in message_sender_names:
        mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        mock_msg_sender.name = name
        msg_senders[name] = mock_msg_sender
    
    def find(name):
        return msg_senders[name]
    
    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.configure_mock(**{"find.side_effect": find})

    return reusable_message_senders

def _create_mock_drone():
    drone = NonCallableMock(spec=MAVROSDrone)
    
    drone_data = Data()
    type(drone).data = PropertyMock(return_value=drone_data)

    gimbal = NonCallableMock(GimbalController)
    drone.gimbal = gimbal

    return drone

def _create_mock_state():
    state = NonCallableMock(spec=BaseState)
    
    state.uav_id = "unit_test_drone"
    state.drone = _create_mock_drone()
    state.drone.system_id = 1
    state.drone.fc_component_id = 1
    state.mqtt = NonCallableMock(spec=MQTTClient)
    state.reusable_message_senders = _create_reusable_message_senders()
    state.message_senders = set()
    state.handlers = MessageHandler()
    state.trajectory = NonCallableMock(spec=TrajectoryGenerator)
    
    return state


def _create_mock_imu(q: Tuple[float, float, float, float]):
    q = Quaternion(q[0], q[1], q[2], q[3])
    imu = NonCallableMock(Imu)
    imu.orientation = q
    return imu

def _create_mock_position(lat, lon, alt):
    position = NonCallableMock(NavSatFix)
    position.latitude = lat
    position.longitude = lon
    position.altitude = alt
    return position


def _create_msg(kind, data):
    return {
        "type": kind,
        "data": data,
    }

class TestGimbal(unittest.TestCase):
    def test_how_we_build_it(self):
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        self.assertNotEqual(gimbal_comp, None)
    
    def test_stare_position_getter_and_setter(self):
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        t = (1.0, 2.0, 3.0)
        gimbal_comp.stare_position = t
        self.assertEqual(gimbal_comp.stare_position, t)
    
    def test_imu_messages(self):
        """
        Test that imu messages update the internal state of the Gimbal component
        """
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        roll = 0
        pitch = 45
        yaw = 90
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        q = Quaternion(q[0], q[1], q[2], q[3])
        imu = NonCallableMock(Imu)
        imu.orientation = q
        msg = {
            "type": "imu",
            "data": imu,
        }
        state.handlers.notify(msg)

        self.assertEqual(gimbal_comp.drone_data.attitude, q)
    
    def test_position_messages(self):
        """
        Test that position messages update the internal state of the Gimbal component
        """
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        position = _create_mock_position(1.0, 2.0, 3.0)
        
        msg = {
            "type": "position",
            "data": position,
        }
        state.handlers.notify(msg)

        self.assertEqual(gimbal_comp.drone_data.position.latitude, 1.0)
        self.assertEqual(gimbal_comp.drone_data.position.longitude, 2.0)
        self.assertEqual(gimbal_comp.drone_data.position.altitude, 3.0)

    def fix_me_test_gimbal_message(self):
        """
        Test that the component sends a command to the gimbal when it's time
        """
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        # The Test Scenario 
        # - stare position (this is at the field)
        stare_pos = Lla(41.714851, -86.241804, 220.5533)
        t = stare_pos.lat, stare_pos.lon, stare_pos.alt
        gimbal_comp.stare_position = t
        self.assertAlmostEqual(gimbal_comp.stare_position, t)
        self.assertAlmostEqual(gimbal_comp._mode, Gimbal.Mode.TRACK_POSITION)

        # - drone position 10 meters south and 10 meters above the stare position
        drone_pos = stare_pos.move_ned(-10, 0.0, -10)
        pos = _create_mock_position(drone_pos.lat, drone_pos.lon, drone_pos.alt)
        msg = _create_msg("position", pos)
        state.handlers.notify(msg)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.latitude, drone_pos.latitude)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.longitude, drone_pos.longitude)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.altitude, drone_pos.altitude)

        # - drone attitude facing north and level with the horizon
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        msg = _create_msg("imu", _create_mock_imu(q))
        state.handlers.notify(msg)
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.x, q[0])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.y, q[1])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.z, q[2])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.w, q[3])
        
        # then send the "gimbal" message and check if the component sent a command to the gimbal
        msg = _create_msg("gimbal", 0.1)
        state.handlers.notify(msg)

        # check that the method was called
        state.drone.gimbal.set_attitude.assert_called_once()

        # check the gimbal component called set_attitude with acceptable args
        args = state.drone.gimbal.set_attitude.call_args.args

        self.assertTrue(len(args) > 1)
        # check the mavlink node
        mavnode = args[1]        
        self.assertEqual(mavnode, MavlinkNode(1, 1))

        # check that the gimbal is set to look with roll = 0, pitch = -45 degrees and yaw = 0
        actual_q = args[0]
        actual_euler = tf.transformations.euler_from_quaternion(actual_q)
        actual_roll, actual_pitch, actual_yaw = actual_euler[0], actual_euler[1], actual_euler[2]

        self.assertAlmostEqual(actual_roll, 0.0)
        self.assertAlmostEqual(math.degrees(actual_pitch), -45.0, 1)
        self.assertAlmostEqual(actual_yaw, 0.0)
    
    def fix_me_test_gimbal_bug_42_camera_looks_the_wrong_way(self):
        """
        Test that the component sends a command to the gimbal when it's time
        """
        state = _create_mock_state()
        gimbal_comp = Gimbal(state)

        # The Test Scenario 
        # - stare position (this is at the field)
        stare_pos = Lla(41.70573086523541, -86.24421841999177, 218.566)
        t = stare_pos.lat, stare_pos.lon, stare_pos.alt
        gimbal_comp.stare_position = t
        self.assertAlmostEqual(gimbal_comp.stare_position, t)
        self.assertAlmostEqual(gimbal_comp._mode, Gimbal.Mode.TRACK_POSITION)

        # - drone position 10 meters south and 10 meters above the stare position
        drone_pos = Lla(41.70618102317103, -86.2436177032699, 238.56639210206723)
        pos = _create_mock_position(drone_pos.lat, drone_pos.lon, drone_pos.alt)
        msg = _create_msg("position", pos)
        state.handlers.notify(msg)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.latitude, drone_pos.latitude)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.longitude, drone_pos.longitude)
        self.assertAlmostEqual(gimbal_comp.drone_data.position.altitude, drone_pos.altitude)

        # - drone attitude facing north and level with the horizon
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        msg = _create_msg("imu", _create_mock_imu(q))
        state.handlers.notify(msg)
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.x, q[0])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.y, q[1])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.z, q[2])
        self.assertAlmostEqual(gimbal_comp.drone_data.attitude.w, q[3])
        
        # then send the "gimbal" message and check if the component sent a command to the gimbal
        msg = _create_msg("gimbal", 0.1)
        state.handlers.notify(msg)

        # check that the method was called
        state.drone.gimbal.set_attitude.assert_called_once()

        # check the gimbal component called set_attitude with acceptable args
        args = state.drone.gimbal.set_attitude.call_args.args

        self.assertTrue(len(args) > 1)
        # check the mavlink node
        mavnode = args[1]        
        self.assertEqual(mavnode, MavlinkNode(1, 1))

        # check that the gimbal is set to look with roll = 0, pitch = -45 degrees and yaw = 0
        actual_q = args[0]
        actual_euler = tf.transformations.euler_from_quaternion(actual_q)
        actual_roll, actual_pitch, actual_yaw = actual_euler[0], actual_euler[1], actual_euler[2]

        self.assertAlmostEqual(actual_roll, 0.0)
        self.assertAlmostEqual(math.degrees(actual_pitch), -45.0, 1)
        self.assertAlmostEqual(actual_yaw, 0.0)
    
    def test_orientation_to_compass_heading_math(self):
        compass = gimbal_math.ChowdhuryMethod.enu_up_angle_to_compass

        def test_compass(input, expected):
            actual = compass(input)
            self.assertEqual(actual, expected)

        test_compass(-1, 91)
        test_compass(1, 89)
        test_compass(179, 271)
        test_compass(-179, 269)
        test_compass(-90, 180)

from dr_onboard_autonomy.gimbal import Gimbal as GimbalIO


def mock_mavsender():
    mav_sender = NonCallableMock()
    mav_sender.system_id = 1
    mav_sender.component_id = 191
    send_event = Event()

    mav_sender.send.side_effect = lambda _: send_event.set()
    return mav_sender, send_event

class TestGimbalIO(unittest.TestCase):

    def testAttitudeProperty(self):
        mav_sender, event = mock_mavsender()

        self.assertFalse(event.is_set())

        mav_sender.send(None)
        self.assertTrue(event.is_set())
        event.clear()

        gimbal_io = GimbalIO(mav_sender)
        self.assertEqual(gimbal_io.attitude, None)

        gimbal_io.start()
        try:
            self.assertEqual(gimbal_io.attitude, None)

            gimbal_io.take_control(gimbal_manager=MavlinkNode(1, 1))
            self.assertEqual(gimbal_io.attitude, None)
            event.clear()

            input_q = (1, 0, 0, 0)
            gimbal_io.set_attitude(input_q, MavlinkNode(1, 1))
            event.wait(3)

            actual_q = gimbal_io.attitude
            expected_q = (1, 0, 0, 0)
        
            self.assertEqual(actual_q, expected_q)

            input_q2 = (1, 2, 3, 4)
            event.clear()
            gimbal_io.set_attitude(input_q2, MavlinkNode(1, 1))
            event.wait(3)
            self.assertEqual(gimbal_io.attitude, input_q2)
        finally:
            gimbal_io.stop(join=True)

