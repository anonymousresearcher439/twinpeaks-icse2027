from typing import List, Set
from queue import Queue
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory

import rospy

from dr_onboard_autonomy.briar_helpers import briarlla_from_ros_position, BriarLla
from dr_onboard_autonomy.models import FCUCopterMode, LlaPosition
from .BaseState import BaseState

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


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
        position: LlaPosition = message['data']
        position: BriarLla = briarlla_from_ros_position(position)
        self.return_channel.put(position)
        return "succeeded"


class AwaitMAVROS(BaseState):
    def __init__(self, reusable_message_senders=None, **kwargs):
        """
        Wait for mavros to come online.
        
        This is useful for when the program is initalizing.
        It's also useful for when things aren't working.
        For example, If this finishes, but AwaitPX4 doesn't, then we know that
        the problem is between PX4 and mavros.
        """
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(reusable_message_senders=reusable_message_senders, **kwargs)

        self.message_senders.add(reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state)
    
    def on_entry(self, _):
        rospy.loginfo("Waiting for MAVROS")
    
    def on_state(self, _):
        rospy.loginfo("MAVROS ready")
        return "succeeded"


class AwaitPX4(BaseState):
    def __init__(self, reusable_message_senders=None, **kwargs):
        """
        Wait for PX4 to come online and send a heartbeat message. This is useful for when the program is initalizing and we need to wait for PX4 to be ready.
        """
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(reusable_message_senders=reusable_message_senders, **kwargs)

        self.message_senders.add(reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state)
    
    def on_entry(self, userdata):
        rospy.loginfo("Waiting for PX4")
    
    def on_state(self, message):
        state: FCUCopterMode = message['data']
        if state.connected:
            rospy.loginfo("PX4 ready")
            return "succeeded"
                


@register_state(default_transitions={"error": "failure"})
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

    def get(self):
        return self.return_channel.get()


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class ReadMessagesAirborne(ReadMessages):
    def __init__(self,
        message_names:List[str]=[],
        return_channel:Queue=None,
        drone=None,
        reusable_message_senders=None,
        mqtt_client=None,
        unwanted_outcomes: Set[str]=set(),
        **kwargs
    ):
        """Hover and wait for at least one of each message to arrive then make
        the messages available via return_channel
        """

        # We need to add some more outcomes to make sure we respond
        # appropriately in case the remote pilot takes action
        all_outcomes = {"succeeded", "error", "human_control", "abort", "rtl"}
        provided_outcomes = kwargs.get("outcomes", [])
        for outcome in provided_outcomes:
            all_outcomes.add(outcome)
        if unwanted_outcomes:
            all_outcomes = all_outcomes - unwanted_outcomes
        kwargs["outcomes"] = list(all_outcomes)

        super().__init__(
            message_names=message_names,
            return_channel=return_channel,
            drone=drone,
            reusable_message_senders=reusable_message_senders,
            mqtt_client=mqtt_client,
            trajectory_class=HoldingTrajectory,
            **kwargs
        )


    def on_entry(self, userdata):
        self.trajectory.start()
        super().on_entry(userdata)
    

