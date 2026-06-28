import time

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


class Searching(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_searching", "cmd_return"]
        super().__init__(**kwargs)
        self.message_senders.add(RepeatTimer("transition_time", 10))
        self.handlers.add_handler("transition_time", self.tx_handler)

    def tx_handler(self, msg):
        number = random.randint(0, 10)
        if number < 5:
            return "succeeded_searching"
        else:
            return "cmd_return"
