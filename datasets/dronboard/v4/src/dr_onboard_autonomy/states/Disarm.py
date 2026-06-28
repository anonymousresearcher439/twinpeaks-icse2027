from dr_onboard_autonomy.states.components import AirLease
from .BaseState import BaseState
from dr_onboard_autonomy.models import FCUState

from dr_onboard_autonomy.state_factory import register_state


@register_state(default_transitions={"error": "failure"})
class Disarm(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_disarm", "error"]
        super().__init__(**kwargs)

        self._pass_through_kwargs = kwargs

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)
        self.air_lease = AirLease(self, cleanup_messages=False)
    
    
    def on_entry(self, userdata):
        self.air_lease.send_land_message()

    def on_state_message(self, message):
        drone_state: FCUState = message["data"]
        is_message_ok = all([message["type"] == "state", drone_state is not None])
        if not is_message_ok: 
            return

        if not drone_state.armed:
            return "succeeded_disarm"
