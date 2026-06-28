import json
import unittest
from typing import Any
from unittest.mock import NonCallableMock, Mock

# from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.briar_helpers import briarlla_from_lat_lon_alt
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import UpdateStarePosition, Gimbal
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator


def mock_message_senders():
    generic_message_sender = NonCallableMock(spec=AbstractMessageSender)
    update_stare_position = NonCallableMock(spec=AbstractMessageSender)
    update_stare_position.name = "update_stare_position"

    def find(name):
        if name == "update_stare_position":
            return update_stare_position
        else:
            return generic_message_sender

    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.find = find
    return reusable_message_senders


def mock_state():
    state = BaseState(
        # air_lease_service=NonCallableMock(spec=AirLeaseService),
        drone=NonCallableMock(spec=MAVROSDrone),
        heartbeat_handler=False,
        local_mqtt_client=NonCallableMock(spec=MQTTClient),
        mqtt_client=NonCallableMock(spec=MQTTClient),
        name="test_state",
        outcomes=["success"],
        reusable_message_senders=mock_message_senders(),
        trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
    )
    return state

def mock_gimbal_driver():
    gimbal_driver = NonCallableMock(spec=Gimbal)
    return gimbal_driver


def mock_message(type: str, data: Any):
    return {
        "type": type,
        "data": data
    }

class TestUpdateStarePosition(unittest.TestCase):
    def test_the_init_method(self):
        state = mock_state()
        gimbal_driver = mock_gimbal_driver()
        component: UpdateStarePosition = UpdateStarePosition(state, gimbal_driver)
        
        self.assertIn(state.reusable_message_senders.find("update_stare_position"), state.message_senders)
        self.assertIn(component.on_update_stare_position, state.handlers.active_handlers["update_stare_position"])
        self.assertEqual(component.gimbal_driver, gimbal_driver)

    def test_on_update_stare_position(self):
        example_payload = """
        {
            "stare_position": {
                "latitude": 41.60699007348136, 
                "longitude": -86.35500108462884,
                "altitude": 229.02
            }
        }
        """
        mqtt_message = NonCallableMock()
        mqtt_message.payload = example_payload.encode("utf-8")
        mqtt_message.topic = "SOME MQTT TOPIC" # This is not used in the method under test
        state = mock_state()
        gimbal_driver = mock_gimbal_driver()
        component: UpdateStarePosition = UpdateStarePosition(state, gimbal_driver)

        message = mock_message("update_stare_position", mqtt_message)
        component.on_update_stare_position(message)

        briarlla = briarlla_from_lat_lon_alt(41.60699007348136, -86.35500108462884, 229.02, is_amsl=True)
        self.assertEqual(gimbal_driver.stare_position, briarlla.ellipsoid.tup)


