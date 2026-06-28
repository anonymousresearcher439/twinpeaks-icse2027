import rospy

from dr_onboard_autonomy.state_factory import register_state

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
        if self.data is not None:
            self.data["arm_position"] = None
            self.message_senders.add(self.reusable_message_senders.find("position"))
            self.handlers.add_handler("position", self.on_position)

    def on_position(self, message):
        if message["data"] is not None:
            self.data["arm_position"] = message["data"]

    def on_entry(self, userdata):
        self._set_disarm_preflight()

        rospy.loginfo("arming")
        is_success = self.drone.arm()
        if is_success is not None and is_success == False:
            rospy.logfatal("could not arm")

    def on_state_message(self, message):
        is_message_ok = all([message["type"] == "state", message["data"] is not None])
        if not is_message_ok:
            return

        if not self.data["arm_position"]:
            return

        drone_state = message["data"]
        if drone_state.armed:
            return "succeeded_armed"

    def _set_disarm_preflight(self, disarm_time: float=300.0):
        """Sets the threhold for disarming the drone after arming without a subsequent transition
        Disarm time in seconds with negative values setting indefinite limit
        """
        disarm_result = self.drone._set_param(
            "COM_DISARM_PRFLT", real_value=disarm_time
        )
        if disarm_result:
            msg = f"Disarm with no transition after arming set to {disarm_time} seconds"
            self.mqtt_client.arming_status_update(msg, "success")
            rospy.loginfo(msg)
        else:
            msg = "Unable to set disarm preflight threshold"
            self.mqtt_client.arming_status_update(msg, "error")
            rospy.logerr(msg)


