from dr_onboard_autonomy.briar_helpers import BriarLla, convert_tuple_to_LlaDict
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.states.components.Setpoint import Setpoint
import rospy

from .BaseState import BaseState


class Offboard(BaseState):
    """This switches the drone to offboard mode

    The 'succeeded' outcome will occur when the drone's mode is "OFFBOARD"
    """

    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(**kwargs)

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)
        self.setpoint_driver = Setpoint(self)

        self.start_pos = None
        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_first_position_message)



    def on_entry(self, userdata):
        rospy.loginfo("switching to offboard mode")
    
    def on_first_position_message(self, message):
        if self.start_pos is not None:
            return
        pos = message["data"]
        pos_tup = pos.latitude, pos.longitude, pos.altitude # Ellipsoidal altitude
        pos_dict = convert_tuple_to_LlaDict(pos_tup)
        self.start_pos = BriarLla(pos_dict, is_amsl=False)

        self.setpoint_driver.start()

    def on_state_message(self, message):
        is_message_ok = all([message["type"] == "state", message["data"] is not None])
        if not is_message_ok:
            return
        
        drone_state = message["data"]
        if drone_state.mode == "OFFBOARD":
            return "succeeded"

