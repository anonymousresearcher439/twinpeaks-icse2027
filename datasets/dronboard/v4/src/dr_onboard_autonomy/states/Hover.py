from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
import rospy

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.models import FCUCopterMode
from dr_onboard_autonomy.state_factory import register_state
from .BaseState import BaseState

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class Hover(BaseState):
    def __init__(self, time_limit=30, **kwargs):
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_hover"], STANDARD_TRANSITIONS_FLYING.keys(), kwargs.get("unwanted_outcomes", set()))
        kwargs['trajectory_class'] = HoldingTrajectory
        super().__init__(**kwargs)
        self.message_senders.add(RepeatTimer("hover_timer", time_limit))
        self.handlers.add_handler("hover_timer", self.on_timer_message)
        self.time_limit = time_limit

    def on_entry(self, userdata):
        self.trajectory.start()

    def on_timer_message(self, message):
        assert message["type"] == "hover_timer"
        assert message["data"] >= self.time_limit
        return "succeeded_hover"

@register_state(default_transitions={
        "error": "failure",
        "failsafe": "Failsafe"
})
class AbortHover(BaseState):
    def __init__(self, **kwargs):
        kwargs.update({
            "outcomes": ["error", "failsafe"],
            "unwanted_outcomes": {"succeeded_hover", "abort", "rtl", "low_battery"},
            "heartbeat_handler": False,
            'trajectory_class': HoldingTrajectory,
        })
        super().__init__(**kwargs)

    on_entry = Hover.on_entry
