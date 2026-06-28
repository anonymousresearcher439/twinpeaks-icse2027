import rospy

from .BaseState import BaseState


class Land(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_land", "error", "human_control"]
        super().__init__(**kwargs)
        self.message_senders.add(self.reusable_message_senders.find("extended_state"))
        self.handlers.add_handler("extended_state", self.on_extended_state_message)

    def on_entry(self, userdata):
        # This puts the drone in land mode...
        is_mode_change_success = self.drone.land()
        rospy.loginfo(f"LAND mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"

    def on_extended_state_message(self, message):
        if "extended_state" not in self.message_data:
            return

        data = self.message_data
        # if drone reports landed_state == LANDED_STATE_ON_GROUND
        # http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
        if data["extended_state"].landed_state == 1:
            return "succeeded_land"
