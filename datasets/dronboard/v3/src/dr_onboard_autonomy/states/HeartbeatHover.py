import rospy

from dr_onboard_autonomy.mavlink import MavState
from dr_onboard_autonomy.states import BaseState

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class HeartbeatHover(BaseState):
    """State to run when heartbeat status is set to hover
    """
    def __init__(self, **kwargs):

        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_hover"], STANDARD_TRANSITIONS_FLYING.keys())
        kwargs["heartbeat_handler"] = False
        super().__init__(**kwargs)

        self.message_senders.add(self.reusable_message_senders.find("heartbeat_status"))
        self.handlers.add_handler("heartbeat_status", self._on_heartbeat_status_message)


    def on_entry(self, userdata):
        self.drone.onboard_heartbeat.mav_state = MavState.CRITICAL
        is_mode_change_success = self.drone.hover()
        rospy.loginfo(f"HeartbeatHover - AUTO.LOITER mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"


    def _on_heartbeat_status_message(self, message):
        data = message['data']
        if data["status"] == "rtl":
            return self._handle_rtl()

        if data["status"] == "continue":
            self.drone.onboard_heartbeat.mav_state = MavState.ACTIVE
            return self._handle_continue()

    
    def _handle_rtl(self):
        rospy.loginfo("HeartbeatHover - returning rtl")
        return "rtl"


    def _handle_continue(self):
        rospy.loginfo("HeartbeatHover - processing 'continue' with succeeded_hover outcome")
        return "succeeded_hover"