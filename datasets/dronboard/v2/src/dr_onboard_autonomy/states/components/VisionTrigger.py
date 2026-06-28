import json
import rospy
from typing import Any, Dict, Union

from dr_onboard_autonomy.models import ALTITUDE_REFERENCE, LlaPosition
from dr_onboard_autonomy.states import BaseState


class VisionTrigger:
    """Component to trigger a state transition when we receive a vision message.
    """

    def __init__(self, state: BaseState):
        state.message_senders.add(state.reusable_message_senders.find("vision"))
        state.handlers.add_handler("vision", self.on_vision)

    def on_vision(self, _):
        return "found"


class TargetPosition:
    """Component to trigger state transition and store a target position when we receive a
    target position
    """
    def __init__(self, state: BaseState):
        self.state = state

        rospy.loginfo("TargetPosition - adding target_position message sender and handler")
        state.message_senders.add(state.reusable_message_senders.find("target_position"))
        state.handlers.add_handler("target_position", self._save_target_position, is_active=False)
        state.handlers.add_handler("target_position", self._trigger_outcome, is_active=True)

    def _save_target_position(self, message: Dict[str, Union[str, Any]]):
        """Stores the received target position so it can be used by downstream states
        This is an inactive handler, so it will not trigger a state transition.
        """
        message_position_data = json.loads(message["data"].payload)
        self.state.drone.data.target_position = LlaPosition(
            latitude=message_position_data["latitude"],
            longitude=message_position_data["longitude"],
            altitude=message_position_data["altitude_amsl"],
            altitude_ref=ALTITUDE_REFERENCE.AMSL
        )

    def _trigger_outcome(self, _) -> str:
        """Triggers the "target_position" outcome
        This is will trigger a state transition
        """
        return "target_position"
