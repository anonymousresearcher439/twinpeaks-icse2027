import smach

from . import BaseState
from dr_onboard_autonomy.states.components import StatusMessage2
from .collections import MessageHandler, MessageQueueImpl

from dr_onboard_autonomy.state_factory import register_state


@register_state(default_transitions={
    "succeeded": "failure",
    "error": "failure",
})
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
        self.message_queue = MessageQueueImpl()
        self.message_senders = set()
        self.handlers = MessageHandler()
        self._is_performance_analysis_mode = kwargs.get("performance_analysis", False)

        # for status messages
        self.status_message = StatusMessage2(self)

        # A good example of how to start receving messages from a message sender
        # and respond to it in a state
        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        

    on_entry = BaseState.on_entry
    execute = BaseState.execute
    execute2 = BaseState.execute2
    _final = BaseState._final
    start_message_senders = BaseState.start_message_senders
    stop_message_senders = BaseState.stop_message_senders
    acknowledge_message = BaseState.acknowledge_message
    on_shutdown = BaseState.on_shutdown
    on_exit = BaseState.on_exit
