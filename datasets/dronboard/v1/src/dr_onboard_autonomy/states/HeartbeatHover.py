import json
from dr_onboard_autonomy.mavlink.HeartbeatSender import MavState
import rospy

from dr_onboard_autonomy.message_senders import MQTTMessageSender
from dr_onboard_autonomy.states import BaseState


class HeartbeatHover(BaseState):
    """State to run when heartbeat status is set to hover
    """
    def __init__(self, **kwargs):
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()

        for other_outcome in ["succeeded_hover", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(heartbeat_handler=False, **kwargs)

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