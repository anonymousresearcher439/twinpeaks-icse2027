import json
import rospy
from typing import Any, Dict, Union

from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict

class VisionApprove:
    """Component to trigger the state to exit with "task_canceled" outcome when
    we receive a cancel message.
    """

    def __init__(self, state: BaseState, outcomes: Dict[str, Any]):
        self.state = state
        self.allow_approve = "vision_approve" in outcomes
        self.allow_denied = "vision_denied" in outcomes
        state.message_senders.add(state.reusable_message_senders.find("vision_approve"))
        state.handlers.add_handler("vision_approve", self.on_vision_approve)
    
    def on_vision_approve(self, msg):
        mqtt_msg: MQTTMessage = msg["data"]
        try:
            payload = json.loads(mqtt_msg.payload.decode("utf-8"))
            # need to catch the decode error and the json decode error
        except UnicodeDecodeError as e:
            rospy.logerr(f"Error decoding UTF-8 of vision_approve message: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            # TODO determine if should we return "error" to exit the program or just ignore the message
            return
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error decoding JSON of vision_approve message: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            return
        
        if "approve" not in payload:
            rospy.logerr(f"vision_approve message missing 'approve' key: {payload}")
            return
        if payload["approve"] and self.allow_approve:
            return "vision_approve"
        elif not payload["approve"] and self.allow_denied:
            return "vision_denied"
