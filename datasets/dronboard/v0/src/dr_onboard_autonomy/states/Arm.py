import rospy

from .BaseState import BaseState


class Arm(BaseState):
    """This state arms the drone.

    The 'error' outcome will occur if drone.arm() returns false
    The 'succeeded' outcome will occur once it receives a state message with armed == True
    """

    def __init__(self, data=None, **kwargs):
        kwargs["outcomes"] = ["succeeded_armed", "error"]
        super().__init__(**kwargs)

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)
        if data is not None:
            self.data = data
            self.data["arm_position"] = None
            self.message_senders.add(self.reusable_message_senders.find("position"))
            self.handlers.add_handler("position", self.on_position)

    def on_position(self, message):
        if message["data"] is not None:
            self.data["arm_position"] = message["data"]

    def on_entry(self, userdata):
        rospy.loginfo("arming")
        is_success = self.drone.arm()
        if is_success is not None and is_success == False:
            rospy.logfatal("could not arm")
            return "error"

    def on_state_message(self, message):
        is_message_ok = all([message["type"] == "state", message["data"] is not None])
        if not is_message_ok:
            return

        if not self.data["arm_position"]:
            return

        drone_state = message["data"]
        if drone_state.armed:
            return "succeeded_armed"


class ArmForce(Arm):
    def on_entry(self, userdata):
        super().on_entry(userdata)
