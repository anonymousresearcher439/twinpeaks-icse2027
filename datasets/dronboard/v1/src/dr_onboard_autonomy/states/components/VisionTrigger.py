import json

import rospy

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import BaseState

from dr_onboard_autonomy.states.components import Gimbal


class VisionTrigger:
    """Component to trigger a state transition when we receive a vision message.
    """

    def __init__(self, state: BaseState):
        state.message_senders.add(state.reusable_message_senders.find("vision"))
        state.handlers.add_handler("vision", self.on_vision)
    
    def on_vision(self, _):
        return "found"