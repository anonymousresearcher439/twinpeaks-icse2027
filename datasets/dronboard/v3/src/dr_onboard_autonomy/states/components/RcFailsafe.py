"""This module holds the RC Failsafe component
"""
import rospy
from mavros_msgs.msg import RCIn

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.models import FCU_MODES_HUMAN_CONTROL, FCUState, FCUCopterMode


class RcFailsafe:
    """Component to trigger the RC Failsafe
    """

    def __init__(self, state: BaseState):
        self.dr_state = state
        self.drone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers
        self.failsafe_modes = FCU_MODES_HUMAN_CONTROL.copy()

        message_sender = self.reusable_message_senders.find("state")
        self.message_senders.add(message_sender)

        self.handlers.add_handler("state", self._on_state_message)
    
    def remove_mode(self, mode: FCUCopterMode):
        """Remove a mode from the failsafe list

        This component will not trigger the failsafe if the pilot changes to one
        of modes in the set: dr_onboard_autonomy.models.FCU_MODES_HUMAN_CONTROL
        But a state you can selectively remove a mode from that set using this method.

        Note this only applies to this instance of the component. Since all states
        create their own instances of the component, any changes made here will not
        affect other states.
        """
        self.failsafe_modes.remove(mode.value)

    def _on_state_message(self, message):
        """Trigger RC failsafe if pilot changes the flight mode
        """
        fcu_state: FCUState = message["data"]

        if fcu_state.mode.value in self.failsafe_modes:
            rospy.logwarn(f"The pilot is taking control. Activating the 'failsafe' fail-safe because the mode changed to {fcu_state.mode.name}")
            return "failsafe"

