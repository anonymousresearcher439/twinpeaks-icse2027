import rospy

from dr_onboard_autonomy.state_factory import register_state
from dr_onboard_autonomy.models import FCUState, TimeStampedLlaPosition
from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState

@register_state(default_transitions={"error": "failure"})
class Arm(BaseState):
    """This state arms the drone.

    The 'error' outcome will occur if drone.arm() returns false
    The 'succeeded' outcome will occur once it receives a state message with armed == True
    """

    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_armed", "error"]
        super().__init__(**kwargs)

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position)

        self.message_senders.add(RepeatTimer("retry_arm", 1.0))
        self.handlers.add_handler("retry_arm", self.on_timer)

        self.message_senders.add(RepeatTimer("stop_trying_to_arm", 600.0))
        self.handlers.add_handler("stop_trying_to_arm", self.on_fatal_timeout)


        self.is_arm_cmd_success = False

    def on_position(self, message):
        pos: TimeStampedLlaPosition = message["data"]

        if message["data"] is not None:
            self.drone.data.arm_position = pos

    def on_entry(self, userdata):
        rospy.loginfo("arming")
        is_success = self.drone.arm()
        self.is_arm_cmd_success = is_success
        if is_success is not None and is_success == False:
            rospy.logwarn("could not arm")

    def on_timer(self, msg):
        if self.is_arm_cmd_success:
            return
        rospy.loginfo("retrying arm")
        self.is_arm_cmd_success = self.drone.arm()
        if not self.is_arm_cmd_success:
            rospy.logwarn("could not arm")

    def on_state_message(self, message):
        drone_state: FCUState = message["data"]

        is_message_ok = all([message["type"] == "state", drone_state is not None])
        if not is_message_ok:
            return

        if not self.drone.data.arm_position:
            return

        if drone_state.armed:
            return "succeeded_armed"

    def on_fatal_timeout(self, msg):
        rospy.logerr("Failed to arm drone")
        return "error"


