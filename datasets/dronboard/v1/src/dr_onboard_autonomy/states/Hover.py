import rospy

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


class Hover(BaseState):
    def __init__(self, time_limit=30, **kwargs):
        kwargs["outcomes"] = ["succeeded_hover", "error", "human_control", "rtl"]
        super().__init__(**kwargs)
        self.message_senders.add(RepeatTimer("hover_timer", time_limit))
        self.handlers.add_handler("hover_timer", self.on_timer_message)
        self.time_limit = time_limit

    def on_entry(self, userdata):
        is_mode_change_success = self.drone.hover()
        rospy.loginfo(f"AUTO.LOITER mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"

    def on_timer_message(self, message):
        assert message["type"] == "hover_timer"
        assert message["data"] >= self.time_limit
        return "succeeded_hover"


class AbortHover(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["error", "human_control"]
        super().__init__(heartbeat_handler=False, **kwargs)

    on_entry = Hover.on_entry
