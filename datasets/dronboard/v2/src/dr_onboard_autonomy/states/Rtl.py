from dr_onboard_autonomy.states import (
    AbortHover,
    BaseState
)

from dr_onboard_autonomy.state_factory import register_state


@register_state(default_transitions={
    "error": "failure",
    "human_control": "HumanControl",
})
class Rtl(BaseState):
    def __init__(self, **kwargs):
        kwargs.update({
            "outcomes": ["error", "human_control"],
            "heartbeat_handler": False,
        })
        super().__init__(**kwargs)
        self._abort_hover = AbortHover(**kwargs)

    def on_entry(self, userdata):
        return(self._abort_hover.execute(userdata=userdata))

