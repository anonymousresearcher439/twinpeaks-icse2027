#!/usr/bin/env python3
import unittest
from unittest.mock import NonCallableMock

from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.models import (
    CopterDrone,
    DATUM_REFERENCE,
    EnuYaw,
    FCUCopterMode,
    LlaPosition,
    NedVelocity
)
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.collections import MessageHandler
from dr_onboard_autonomy.states.components import Setpoint
from mock_types import mock_copter_drone, mock_message_sender_state_message


def _create_reusable_message_senders():
    message_sender_names = [
        "position",
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

def _create_mock_state():
    state = NonCallableMock(spec=BaseState)

    state.uav_id = "unit_test_drone"
    state.drone: CopterDrone = mock_copter_drone()

    state.mqtt = NonCallableMock(spec=MQTTClient)
    state.reusable_message_senders = _create_reusable_message_senders()
    state.message_senders = set()
    state.handlers = MessageHandler()
    state.data = {}

    return state


class TestSetpoint(unittest.TestCase):
    def test_how_we_build_it(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)

        self.assertNotEqual(setpoint_comp, None)
    
    def test_start_with_no_location(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)

        setpoint_comp.start()
        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)
        self.assertEqual(setpoint_comp._send_count, 0)
        # when started without a known location, the default setpoint should be
        # velocity= 0,0,0
        self.assertEqual(setpoint_comp.velocity, (0,0,0))
        self.assertEqual(setpoint_comp.lla, None)
        self.assertEqual(setpoint_comp.ned, None)
    
    @unittest.skip("Setpoint sender now uses velocity=0,0,0 as it's starting value (it no longer uses the drone's current pos)")
    def test_start_with_location(self):
        state = _create_mock_state()
        l = state.drone.data.location
        l['latitude'] = 1.0
        l['longitude'] = 2.0
        l['altitude'] = 3.0

        setpoint_comp = Setpoint(state)

        setpoint_comp.start()
        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)
        self.assertEqual(setpoint_comp._send_count, 0)
        self.assertEqual(setpoint_comp.velocity, None)
        # when started with a known location, the lla setpoint should be set to the current position
        self.assertEqual(setpoint_comp.lla, (1.0, 2.0, 3.0))
        self.assertEqual(setpoint_comp.ned, None)

    def test_stop(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)

        setpoint_comp.start()
        setpoint_comp.stop()
        self.assertEqual(setpoint_comp._state, Setpoint.State.STOPPED)

    def test_lla_prop_set_as_tuple(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        lla = (1.0, 2.0, 3.0)

        setpoint_comp.lla = lla
        self.assertEqual(
            setpoint_comp.lla,
            LlaPosition(lla[0], lla[1], lla[2], datum_ref=DATUM_REFERENCE.AMSL)
        )

    def test_lla_prop_set_as_LlaPosition(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        lla = LlaPosition(1.0, 2.0, 3.0, DATUM_REFERENCE.AMSL)

        setpoint_comp.lla = lla
        self.assertEqual(
            setpoint_comp.lla,
            lla
        )
    
    def test_ned_prop(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        ned = (1.0, 2.0, 3.0)

        setpoint_comp.ned = ned       
        self.assertEqual(setpoint_comp.ned, ned)

    def test_velocity_prop(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        velocity = (1.0, 2.0, 3.0)

        setpoint_comp.velocity = velocity       
        self.assertEqual(setpoint_comp.velocity, velocity)

    def test_yaw_prop(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        yaw = 1.0

        setpoint_comp.yaw = yaw
        self.assertEqual(setpoint_comp.yaw, yaw)

    def test_that_setpoint_handler_is_properly_registered(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        # the setpoint handler should have called send_setpoint when we don't have a defualt yaw value
        self.assertFalse(state.drone._send_ned_velocity_setpoint.called) 

        # now set a default yaw
        setpoint_comp._default_yaw = 0.0
        state.handlers.notify(msg)
        # so the send_setpoint method should have been called
        state.drone._send_ned_velocity_setpoint.assert_called_once_with(ned_velocity=(0.0, 0.0, 0.0), yaw=0.0)


    def test_that_it_can_send_an_lla_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        pos = (1.0, 2.0, 3.0)
        default_yaw = EnuYaw(2.17)

        setpoint_comp.lla = pos
        setpoint_comp._default_yaw = default_yaw

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_lla_setpoint.assert_called_once_with(
            lla=LlaPosition(
                latitude=pos[0],
                longitude=pos[1],
                altitude=pos[2],
                datum_ref=DATUM_REFERENCE.AMSL
            ),
            yaw=default_yaw
        )

    def test_that_it_can_send_a_ned_pos_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        pos = (1.0, 2.0, 3.0)

        setpoint_comp.ned = pos
        setpoint_comp._default_yaw = 0.0

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_ned_setpoint.assert_called_once_with(ned_position=pos, yaw=0.0)


    def test_that_it_can_send_a_velocity_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        v = (1.0, 2.0, 3.0)
        setpoint_comp.velocity = v
        setpoint_comp._default_yaw = 0.0

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_ned_velocity_setpoint.assert_called_once_with(ned_velocity=v, yaw=0.0)

    def test_that_it_can_send_an_lla_with_yaw_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        pos = (1.0, 2.0, 3.0)
        yaw = EnuYaw(3.14)

        setpoint_comp.lla = pos
        setpoint_comp.yaw = yaw

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_lla_setpoint.assert_called_once_with(
            lla=LlaPosition(
                latitude=pos[0],
                longitude=pos[1],
                altitude=pos[2],
                datum_ref=DATUM_REFERENCE.AMSL
            ),
            yaw=yaw
        )


    def test_that_it_can_send_an_LlaPosition(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        lla_pos = LlaPosition(4.0, 7.0, 5.0, DATUM_REFERENCE.AMSL)
        yaw = EnuYaw(3.14)

        setpoint_comp.lla = lla_pos
        setpoint_comp.yaw = yaw

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_lla_setpoint.assert_called_once_with(
            lla=lla_pos,
            yaw=yaw
        )


    def test_that_it_can_send_a_ned_pos_with_yaw_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        pos = (1.0, 2.0, 3.0)

        setpoint_comp.ned = pos
        setpoint_comp.yaw = 2.0

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_ned_setpoint.assert_called_once_with(ned_position=pos, yaw=2.0)


    def test_that_it_can_send_a_velocity_with_yaw_setpoint(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        v = (1.0, 2.0, 3.0)

        setpoint_comp.velocity = v
        setpoint_comp.yaw = 3.0

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_ned_velocity_setpoint.assert_called_once_with(ned_velocity=v, yaw=3.0)


    def test_that_it_can_send_a_velocity_setpoint_then_an_lla_setpoint_then_an_lla_with_no_yaw(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)
        setpoint_comp.start()

        v = NedVelocity(1.0, 2.0, 3.0)
        yaw = EnuYaw(3.0)

        setpoint_comp.velocity = v
        setpoint_comp.yaw = yaw

        msg = {
            "type": "setpoint",
            "data": 0.1
        }
        state.handlers.notify(msg)
        state.drone._send_ned_velocity_setpoint.assert_called_once_with(ned_velocity=v, yaw=yaw)

        pos = (4.0, 5.0, 6.0)
        setpoint_comp.lla = pos
        state.handlers.notify(msg)
        state.drone._send_lla_setpoint.assert_called_with(
            lla=LlaPosition(
                latitude=pos[0],
                longitude=pos[1],
                altitude=pos[2],
                datum_ref=DATUM_REFERENCE.AMSL
            ),
            yaw=yaw
        )

        setpoint_comp.yaw = None
        default_yaw = EnuYaw(0.0)
        setpoint_comp._default_yaw = default_yaw
        state.handlers.notify(msg)
        state.drone._send_lla_setpoint.assert_called_with(
            lla=LlaPosition( 
                latitude=pos[0],
                longitude=pos[1],
                altitude=pos[2],
                datum_ref=DATUM_REFERENCE.AMSL
            ),
            yaw=default_yaw
        )


    def test_that_it_can_switch_to_offboard_mode(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)

        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)
        setpoint_comp.start()
        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)

        state_msg = mock_message_sender_state_message(mode=FCUCopterMode.LOITER)

        state.handlers.notify(state_msg)
        self.assertEqual(setpoint_comp._state, Setpoint.State.STARTING)
        self.assertEqual(setpoint_comp._send_count, 0)

        setpoint_msg = {
            "type": "setpoint",
            "data": 0.1
        }

        for i in range(35):
            self.assertEqual(setpoint_comp._send_count, i)
            self.assertEqual(setpoint_comp._state, Setpoint.State.STARTING)
            state.handlers.notify(setpoint_msg)
            state.drone.data.mode = FCUCopterMode.UNKNOWN
            state.handlers.notify(state_msg)
            self.assertEqual(setpoint_comp._send_count, i + 1)

        self.assertEqual(setpoint_comp._state, Setpoint.State.RUNNING)
        state.drone._set_fcu_mode.assert_called_once_with(FCUCopterMode.OFFBOARD)

    def test_that_it_can_resume_offboard_mode(self):
        state = _create_mock_state()
        setpoint_comp = Setpoint(state)

        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)
        setpoint_comp.start()
        self.assertEqual(setpoint_comp._state, Setpoint.State.UNKNOWN)

        state_msg = mock_message_sender_state_message(mode=FCUCopterMode.OFFBOARD)

        state.handlers.notify(state_msg)
        self.assertEqual(setpoint_comp._state, Setpoint.State.RUNNING)
        self.assertEqual(setpoint_comp._send_count, 0)
