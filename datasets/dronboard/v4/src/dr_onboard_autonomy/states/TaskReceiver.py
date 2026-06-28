from dataclasses import dataclass
import json
from typing import List, TypedDict, Union
import queue
from dr_onboard_autonomy.message_senders import RepeatTimer
import rospy

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.models.kinematics import LlaPosition
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from .BaseState import BaseState

from jsonc_parser.parser import JsoncParser
from jsonc_parser.errors import ParserError




# @register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class TaskReceiver(BaseState):
    TIMER_MESSAGE_NAME = "send_ready_message"
    def __init__(self, **kwargs):

        kwargs["outcomes"] = self._process_outcomes(kwargs, ["new_task", "end_task_loop"], STANDARD_TRANSITIONS_FLYING.keys())

        super().__init__(trajectory_class=HoldingTrajectory, **kwargs)

        self.READY_MESSAGE_MQTT_TOPIC = f"drone/{self.drone.uav_name}/task/ready"
        self.TASK_OUTCOME_MQTT_TOPIC = f"drone/{self.drone.uav_name}/task/outcome"

        self.return_channel = queue.Queue()

        message_senders = [
            ("new_task", self._new_task),
            ("end_task_loop", self._end_task_loop),
        ]
        self._previous_task_outcome = kwargs.get("previous_task_outcome", False)
        
        self.ready_msg = None
        self.message_senders.add(RepeatTimer(TaskReceiver.TIMER_MESSAGE_NAME, 1.0))
        self.handlers.add_handler(TaskReceiver.TIMER_MESSAGE_NAME, self.send_ready_message)
        self.ready_message_count = 0
        for (name, callback) in message_senders:
            msg_sender = self.reusable_message_senders.find(name)
            self.message_senders.add(msg_sender)
            self.handlers.add_handler(name, callback)
    
    def on_entry(self, userdata):
        assert self.mqtt_client, "MQTT client must be set before entering TaskReceiver state"
        current_pos: LlaPosition = self.drone.data.location.get_position()
        current_pos = current_pos.to_amsl()
        current_pos: BriarLla = BriarLla.from_args(current_pos.latitude, current_pos.longitude, current_pos.altitude, is_amsl=True)
        # send a message to the GCS to let it know we're ready to receive a tasks
        self.ready_msg = {
            "name": self.name,
            "uavid": self.drone.uav_name,
            "timestamp": self.mqtt_client.timestamp(),
            "position": current_pos.amsl.dict,
        }
        
        # Make sure we hover in place
        self.trajectory.start()
    
    def send_ready_message(self, msg):
        if self.ready_message_count % 3 == 0:
            rospy.loginfo("Sending ready message")
            self.mqtt_client.publish(self.READY_MESSAGE_MQTT_TOPIC, self.ready_msg, qos=1)
        
        if self.ready_message_count % 3 == 2 and self._previous_task_outcome:
            rospy.loginfo("Sending task outcome report")
            self.mqtt_client.publish(self.TASK_OUTCOME_MQTT_TOPIC, self._previous_task_outcome, qos=1)
        self.ready_message_count += 1
    
    def get(self) -> str:
        """Gets the task task from the return channel
        Raises: Empty if the channel is empty
        Returns: The task as a string
        """
        return self.return_channel.get(block=False)
    
    def _new_task(self, msg):
        assert self.mqtt_client
        mqtt_msg: 'MQTTMessage' = msg['data']
        try:
            task_json = mqtt_msg.payload.decode('utf-8')
            task_spec = JsoncParser.parse_str(task_json)
            task_id = task_spec.get("task_id")

            if task_id is None:
                rospy.logwarn(f"Received a Task that does not contain a task_id: {task_json}")
                return "new_task"
            
            previous_tasks = self.data.get("task_ids", set())
            if task_id in previous_tasks:
                rospy.loginfo(f"Task {task_spec.get('task_id')} already executed. Skipping.")
                return
            else:
                self.return_channel.put(task_json)
                return "new_task"
        except ParserError as json_error:
            rospy.logerr(f"Error decoding task message: {json_error}\n'")
            lines = [
                f"MQTT Message Payload: '{mqtt_msg.payload}'",
                f"Error: {json_error}",
            ]
            err_msg = "\n".join(lines)
            subject = "Could not parse the task. Please send tasks as JSON with comments"
            rospy.logerr(err_msg)
            self.mqtt_client.send_error(subject, err_msg)
            # return "error"
        except Exception as e:
            subject = "utf-8 decoding error"
            err_msg = f"Error decoding task message: {e}\nMessage Payload:'{mqtt_msg.payload}'"
            rospy.logerr(err_msg)
            self.mqtt_client.send_error(subject, err_msg)

            # return "error"
    
    def _end_task_loop(self, msg):
        mqtt_msg: 'MQTTMessage' = msg['data']
        # TODO should we check the payload?
        return "end_task_loop"

from paho.mqtt.client import MQTTMessage




import traceback
