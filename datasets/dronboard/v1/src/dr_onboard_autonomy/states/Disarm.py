from .BaseState import BaseState
from dr_onboard_autonomy.states import AirLeaser


class Disarm(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_disarm", "error"]
        super().__init__(**kwargs)

        self._pass_through_kwargs = kwargs

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)
    
    
    def on_entry(self, userdata):
        air_leaser = AirLeaser(
            hold_position=False,
            **self._pass_through_kwargs
        )

        outcome = air_leaser.land()
        if outcome != "succeeded":
            return outcome


    def on_state_message(self, message):
        is_message_ok = all([message["type"] == "state", message["data"] is not None])
        if not is_message_ok:
            return
        drone_state = message["data"]
        if not drone_state.armed:
            return "succeeded_disarm"
