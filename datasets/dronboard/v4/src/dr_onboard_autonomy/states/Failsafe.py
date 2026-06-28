from typing import Dict, TypedDict
import smach
from paho.mqtt.client import MQTTMessage
import rospy

from . import BaseState
from dr_onboard_autonomy.states.components import StatusMessage
from .collections import MessageHandler, MessageQueueImpl

from dr_onboard_autonomy.state_factory import register_state


@register_state(default_transitions={
    "succeeded": "mission_completed", # we want to go to mission completed state in case Ground Control sends the reset message
    "error": "failure",
})
class Failsafe(smach.State):
    """This state runs when a failsafe is triggered. While in this state, dr-onboard monitors the drone.

    There is no way to recover from this state. The only way to exit this state is to shut down the program.

    The 'error' outcome will occur if the rospy sends the shutdown message. No other outcomes are possible.
    """

    def __init__(
        self, drone=None, reusable_message_senders=None, mqtt_client=None, local_mqtt_client=None, **kwargs
    ):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(outcomes=kwargs["outcomes"])
        self.name = "Failsafe"
        self.args = kwargs
        self.drone = drone
        self.reusable_message_senders = reusable_message_senders
        self.mqtt_client = mqtt_client
        self.local_mqtt_client = local_mqtt_client
        self.mqtt_cloud = kwargs.get("mqtt_cloud", None)
        self.message_data = {}
        self.message_queue = MessageQueueImpl()
        self.message_senders = set()
        self.handlers = MessageHandler()
        self._is_performance_analysis_mode = kwargs.get("performance_analysis", False)
        self.air_lease_protocol_fsm = kwargs.get("air_lease_protocol_fsm", None)
        self.data = kwargs.get('data', {})

        # for status messages
        self.status_message = StatusMessage(self)

        # A good example of how to start receving messages from a message sender
        # and respond to it in a state
        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        shutdown_sender = self.reusable_message_senders.find("reset")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("reset", self.on_reset)

    
    def on_reset(self, msg: Dict[str, MQTTMessage]) -> None:
        """Reset the state."""
        rospy.loginfo("Received RESET message. Exiting Failsafe mode.")
        return "succeeded"

    on_entry = BaseState.on_entry
    execute = BaseState.execute
    execute2 = BaseState.execute2
    _final = BaseState._final
    start_message_senders = BaseState.start_message_senders
    stop_message_senders = BaseState.stop_message_senders
    acknowledge_message = BaseState.acknowledge_message
    on_shutdown = BaseState.on_shutdown
    on_exit = BaseState.on_exit
    update_drone_data = BaseState.update_drone_data

class ResetMessage(TypedDict):
    """Message type for resetting the state."""
    data: MQTTMessage
