import time

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


class OnGround(BaseState):
    """This state does almost nothing until the the timer goes off
    
    It still does all the standard background work (like sending status messages)
    """
    def __init__(self, **kwargs):
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["usr_initiated_preflight", "error"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(**kwargs)

        self.message_senders.add(RepeatTimer("transition_timer", 30))
        self.handlers.add_handler("transition_timer", self.on_timer)

    def on_timer(self, msg):
        return "usr_initiated_preflight"
