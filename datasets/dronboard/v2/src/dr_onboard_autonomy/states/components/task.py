import json
import rospy
from typing import Any, Dict, Union

from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict

class TaskCanceledTrigger:
    """Component to trigger the state to exit with "task_canceled" outcome when
    we receive a cancel message.
    """

    def __init__(self, state: BaseState, task_id: Union[int, str]):
        self.task_id = task_id
        state.message_senders.add(state.reusable_message_senders.find("cancel_current_task"))
        state.handlers.add_handler("cancel_current_task", self.on_cancel_message)
    
    def on_cancel_message(self, msg):
        mqtt_msg: MQTTMessage = msg["data"]
        try:
            payload = json.loads(mqtt_msg.payload)
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error decoding cancel message: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            # TODO determine if should we return "error" to exit the program or just ignore the message
            #return "error"
        
        if not self.is_valid_task_id():
            # in this case we were initialized with an invalid task_id so just
            # cancel the task no matter what
            # TODO handle this case better
            return "task_canceled" 
        
        if self.task_id == payload.get("task_id"):
            # otherwise we only cancel the task if the task_id matches
            return "task_canceled"
    
    def is_valid_task_id(self):
        """The task_id should be an int or a string.
        If it's None, it's not valid.
        If it's an int, it should be non-negative.
        If it's a string, it should not be empty.
        If the task_id is anything else, then it's not valid.
        """
        if self.task_id is None:
            return False
        if isinstance(self.task_id, int):
            # in python a bool is a subclass of int
            # so lets make sure we're not getting a bool
            return self.task_id >= 0 and not isinstance(self.task_id, bool)
        if isinstance(self.task_id, str):
            return bool(self.task_id)
        return False
        