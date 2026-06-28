import rospy

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.models import FCUCopterMode
from dr_onboard_autonomy.state_factory import register_state
from .BaseState import BaseState

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class Hover(BaseState):
    def __init__(self, time_limit=30, **kwargs):
        kwargs["outcomes"] = ["succeeded_hover", "abort", "error", "human_control", "rtl"]
        super().__init__(**kwargs)
        self.message_senders.add(RepeatTimer("hover_timer", time_limit))
        self.handlers.add_handler("hover_timer", self.on_timer_message)
        self.time_limit = time_limit

    def on_entry(self, userdata):
        is_mode_change_success = self.drone.set_fcu_mode(FCUCopterMode.LOITER)
        rospy.loginfo(f"LOITER mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"

    def on_timer_message(self, message):
        assert message["type"] == "hover_timer"
        assert message["data"] >= self.time_limit
        return "succeeded_hover"

@register_state(default_transitions={
        "error": "failure",
        "human_control": "HumanControl"
})
class AbortHover(BaseState):
    def __init__(self, **kwargs):
        kwargs.update({
            "outcomes": ["error", "human_control"],
            "heartbeat_handler": False,
        })
        super().__init__(**kwargs)

    on_entry = Hover.on_entry
