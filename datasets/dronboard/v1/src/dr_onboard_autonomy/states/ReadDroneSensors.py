from typing import List
from queue import Queue

import rospy
from sensor_msgs.msg import NavSatFix

from dr_onboard_autonomy.briar_helpers import briarlla_from_ros_position, BriarLla

from .BaseState import BaseState



class ReadPosition(BaseState):
    def __init__(self, return_channel:Queue=None, drone=None, reusable_message_senders=None, mqtt_client=None, **kwargs):
        """
        Reads the drone's position and sends it to the return_channel as a BriarLla
        
        return_channel should be a Queue
        """
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(drone=drone, reusable_message_senders=reusable_message_senders, mqtt_client=mqtt_client, **kwargs)
        if return_channel is None:
            return_channel = Queue()
        self.return_channel = return_channel

        self.message_senders.add(reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position)
    
    def on_entry(self, userdata):
        rospy.loginfo("Reading the aircraft's position")
    
    def on_position(self, message):
        position: NavSatFix = message['data']
        position: BriarLla = briarlla_from_ros_position(position)
        self.return_channel.put(position)
        return "succeeded"
        

class ReadMessages(BaseState):
    def __init__(self, message_names:List[str]=[], return_channel:Queue=None, drone=None, reusable_message_senders=None, mqtt_client=None, **kwargs):
        """
        Given a list of message sender names, this will collect one of ever message into a dictionary and send the result to the return_channel.
        
        message_names is a list of strings. Each string must be a name found in the reusable_message_senders collection
        return_channel should be a Queue. The resulting dictionary will be 'put' on this queue
        """
        all_outcomes = {"succeeded", "error"}
        provided_outcomes = kwargs["outcomes"]
        for outcome in provided_outcomes:
            all_outcomes.add(outcome)
        
        kwargs["outcomes"] = list(all_outcomes)
        super().__init__(drone=drone, reusable_message_senders=reusable_message_senders, mqtt_client=mqtt_client, **kwargs)
        if return_channel is None:
            return_channel = Queue()
        self.return_channel = return_channel
        self.message_names = set(message_names)
        for name in self.message_names:
            self.message_senders.add(reusable_message_senders.find(name))
            self.handlers.add_handler(name, self.on_message)
        
        self.result = {}
    
    
    def on_message(self, message):
        name = message['type']
        data = message['data']
        
        self.result[name] = data

        for name in self.message_names:
            if name not in self.result:
                return

        self.return_channel.put(self.result)
        return "succeeded"