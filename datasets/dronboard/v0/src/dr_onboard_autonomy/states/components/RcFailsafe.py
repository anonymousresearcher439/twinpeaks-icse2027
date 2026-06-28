"""This module holds the RC Failsafe component
"""
import rospy
from mavros_msgs.msg import RCIn

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.states import BaseState


def get_rc_channel_value(rcin: RCIn, channel_number: int) -> int:
    """Return the channel PWM value from the RCIn object.
    Args:
        rcin: RCIn the RC inputs from mavros
        channel_number: the channel to read
    """
    channel_index = channel_number - 1
    return rcin.channels[channel_index]


class RcFailsafe:
    """Component to trigger the RC Failsafe
    """

    def __init__(self, state: BaseState):
        self.dr_state = state
        self.drone: MAVROSDrone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers

        for sender_name in ["state", "rcin"]:
            message_sender = self.reusable_message_senders.find(sender_name)
            self.message_senders.add(message_sender)

        self.handlers.add_handler("state", self._on_state_message)
        self.handlers.add_handler("rcin", self._on_rcin_message)

    def _on_state_message(self, message):
        """Trigger RC failsafe if pilot changes the flight mode
        """
        user_control_modes = [
            "ALTCTL",
            "AUTO.RTL",
            "POSCTL",
            "STABILIZED",
        ]
        if message["data"].mode in user_control_modes:
            rospy.logwarn(f"The pilot is taking control. Activating the 'human_control' fail-safe because the mode changed to {message['data'].mode}")
            return "human_control"

    def _on_rcin_message(self, message):
        """Trigger the RC failsafe if pilot commands the drone to land using the
        RC unit

        We need this check to distinguish between the pilot entering land mode
        and dr_onboard entering it.
        """
        rcin: RCIn = message['data']
        chan5_raw = get_rc_channel_value(rcin, 5)
        # Land mode
        if 1160 <= chan5_raw and chan5_raw <= 1320:
            rospy.logwarn(f"The pilot is taking control. Activating the 'human_control' fail-safe because the RC channel 5 changed to {chan5_raw}")
            return "human_control"
