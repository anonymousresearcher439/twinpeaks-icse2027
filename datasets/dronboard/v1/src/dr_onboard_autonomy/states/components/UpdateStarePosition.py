import json

import rospy

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import BaseState

from dr_onboard_autonomy.states.components import Gimbal


class UpdateStarePosition:
    """Component to update the gimbal stare position mid-flight.
    """

    def __init__(self, state: BaseState, gimbal_driver: Gimbal):
        state.message_senders.add(state.reusable_message_senders.find("update_stare_position"))
        state.handlers.add_handler("update_stare_position", self.on_update_stare_position)
        self.gimbal_driver = gimbal_driver
    
    def on_update_stare_position(self, message):
        message_payload = message["data"].payload.decode("utf-8")
        try:
            msg = json.loads(message_payload)
            stare_position = BriarLla(msg["stare_position"], is_amsl=True)
            self.gimbal_driver.stare_position = stare_position.ellipsoid.tup
            rospy.loginfo("Updating stare position to {}".format(stare_position.amsl.tup))
        except json.decoder.JSONDecodeError as e:
            rospy.logerr(f"Error decoding JSON: {e}")
            rospy.logerr("Message payload: " + message_payload)