from dataclasses import dataclass
import json
from typing import List, TypedDict, Union
import queue
import rospy

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from .BaseState import BaseState


# @register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class TaskReceiver(BaseState):
    def __init__(self, **kwargs):
        outcomes_set = {
            "new_task",
            "end_task_loop",
            "error",
            "human_control",
            "abort",
            "rtl",
        }
        if "outcomes" in kwargs:
            outcomes_set.update(kwargs["outcomes"])
        kwargs["outcomes"] = list(outcomes_set)

        super().__init__(trajectory_class=HoldingTrajectory, **kwargs)

        self.READY_MESSAGE_MQTT_TOPIC = f"drone/{self.drone.uav_name}/task/ready"

        self.return_channel = queue.Queue()

        message_senders = [
            ("new_task", self._new_task),
            ("end_task_loop", self._end_task_loop),
        ]
        for (name, callback) in message_senders:
            msg_sender = self.reusable_message_senders.find(name)
            self.message_senders.add(msg_sender)
            self.handlers.add_handler(name, callback)
    
    def on_entry(self, userdata):
        # send a message to the GCS to let it know we're ready to receive a tasks
        msg = {
            "uavid": self.drone.uav_name,
            "timestamp": self.mqtt_client.timestamp(),
        }
        self.mqtt_client.publish(self.READY_MESSAGE_MQTT_TOPIC, msg, qos=1)
        # Make sure we hover in place
        self.trajectory.start()
    
    
    def get(self) -> Union[str, bytes]:
        return self.return_channel.get(block=False)
    
    def _new_task(self, msg):
        mqtt_msg: 'MQTTMessage' = msg['data']
        try:
            task_json = mqtt_msg.payload.decode('utf-8')
            json.loads(task_json)
            self.return_channel.put(task_json)
            return "new_task"
        except json.JSONDecodeError as json_error:
            rospy.logerr(f"Error decoding task message: {json_error}\n'")
            lines = [
                f"MQTT Message Payload: '{mqtt_msg.payload}'",
                f"JSONDecodeError: '{str(json_error)}'",
                f"doc:    '{json_error.doc}'",
                f"msg:    '{json_error.msg}'",
                f"pos:    {json_error.pos}",
                f"lineno: {json_error.lineno}",
                f"colno:  {json_error.colno}",  
            ]
            err_msg = "\n".join(lines)
            rospy.logerr(err_msg)
            # return "error"
        except Exception as e:
            rospy.logerr(f"Error decoding task message: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            # return "error"
    
    def _end_task_loop(self, msg):
        mqtt_msg: 'MQTTMessage' = msg['data']
        # TODO should we check the payload?
        return "end_task_loop"

from paho.mqtt.client import MQTTMessage




import traceback
