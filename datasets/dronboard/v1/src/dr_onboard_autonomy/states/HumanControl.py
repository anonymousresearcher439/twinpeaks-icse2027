import rospy
import smach

from . import BaseState
from .collections import MessageHandler, MessageQueue
from dr_onboard_autonomy.message_senders import RepeatTimer


class HumanControl(smach.State):
    """This state let's the humans fly the drone using the RC transmitter.

    The 'error' outcome will occur if the rospy sends the shutdown message
    The 'succeeded' outcome will occur if the drone lands
    """

    def __init__(
        self, drone=None, reusable_message_senders=None, mqtt_client=None, **kwargs
    ):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(outcomes=kwargs["outcomes"])
        self.name = "HumanControl"
        self.args = kwargs
        self.drone = drone
        self.reusable_message_senders = reusable_message_senders
        self.mqtt_client = mqtt_client
        self.message_data = {}
        self.message_queue = MessageQueue()
        self.message_senders = set()
        self.handlers = MessageHandler()

        # To send the drone's status over MQTT  we need to make status messages and to make that we
        # need these message senders:
        self.data_senders = {"position", "relative_altitude", "state", "battery"}
        for message_type in self.data_senders:
            message_sender = self.reusable_message_senders.find(message_type)
            self.message_senders.add(message_sender)
            self.handlers.add_handler(message_type, self.on_data_update_message)

        # A good example of how to start receving messages from a message sender
        # and respond to it in a state
        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        self.send_data_timer = RepeatTimer("send_data_timer", 1)
        self.message_senders.add(self.send_data_timer)
        self.handlers.add_handler("send_data_timer", self.on_send_data)

    on_entry = BaseState.on_entry
    execute = BaseState.execute
    execute2 = BaseState.execute2
    start_message_senders = BaseState.start_message_senders
    stop_message_senders = BaseState.stop_message_senders
    acknowledge_message = BaseState.acknowledge_message
    on_shutdown = BaseState.on_shutdown
    on_data_update_message = BaseState.on_data_update_message
    on_send_data = BaseState.on_send_data
    on_exit = BaseState.on_exit
