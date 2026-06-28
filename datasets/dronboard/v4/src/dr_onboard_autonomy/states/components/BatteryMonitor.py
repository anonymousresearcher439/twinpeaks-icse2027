import json
from dr_onboard_autonomy.models.drone import Battery
import rospy
from typing import Any, Dict, Union

from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict

class BatteryMonitor:
    """Component to trigger the state to exit with "task_canceled" outcome when
    we receive a cancel message.
    """

    def __init__(self, state: BaseState):
        self.state = state
        state.message_senders.add(state.reusable_message_senders.find("battery"))
        state.handlers.add_handler("battery", self.on_battery)
    
    def on_battery(self, msg):
        battery_msg: Battery = msg["data"]
        if battery_msg.level < 0.33:
            return "low_battery"
