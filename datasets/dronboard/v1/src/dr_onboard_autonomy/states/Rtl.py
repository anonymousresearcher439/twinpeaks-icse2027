from dr_onboard_autonomy.states import (
    AbortHover,
    BaseState
)

class Rtl(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["error", "human_control"]
        super().__init__(heartbeat_handler=False, **kwargs)
        self._abort_hover = AbortHover(**kwargs)


    def on_entry(self, userdata):
        return(self._abort_hover.execute(userdata=userdata))

