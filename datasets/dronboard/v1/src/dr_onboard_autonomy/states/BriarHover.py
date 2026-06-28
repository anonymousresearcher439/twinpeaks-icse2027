from typing import Optional

import rospy

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    LlaDict,
    convert_LlaDict_to_tuple,
    amsl_to_ellipsoid,
    ellipsoid_to_amsl
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, UpdateStarePosition
from dr_onboard_autonomy.states.components.trajectory import YawTrajectory

from .BaseState import BaseState


_DEFAULT_STARE_POS = LlaDict(latitude=0.0, longitude=0.0, altitude=0.0)


class BriarHover(BaseState):
    def __init__(self, data=None, hover_time:float=10, stare_position:Optional[LlaDict]=_DEFAULT_STARE_POS, **kwargs):
        """
        Hovers the drone while pointing the camera.

        Args:
            hover_time: float in seconds. How long the drone will hover.
            stare_position: where the camera looks. Alt is AMSL.
        """
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_hover", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(trajectory_class=YawTrajectory, **kwargs)

        self.data = data
        self.hover_time = hover_time
        self.stare_position = None
        self.hover_pos = None

        if stare_position is not None:
            stare_position: BriarLla = BriarLla(stare_position, is_amsl=True)
            self.stare_position = stare_position.ellipsoid.tup
            self.gimbal_driver = Gimbal(self)
            self.stare_position_updater = UpdateStarePosition(self, self.gimbal_driver)
        
        self.setpoint_driver = self.trajectory.setpoint_driver

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self.message_senders.add(RepeatTimer("hover_timer", hover_time))
        self.handlers.add_handler("hover_timer", self.on_hover_timer)

    def on_entry(self, userdata):
        self.setpoint_driver.start()
        
        if self.stare_position is not None:
            self.gimbal_driver.start()
            self.gimbal_driver.stare_position = self.stare_position
    
    def on_position_message(self, message):
        if self.hover_pos != None:
            return
        pos = message["data"]
        ellipsoidal_lla_tuple = pos.latitude, pos.longitude, pos.altitude
        target_lla = ellipsoid_to_amsl(ellipsoidal_lla_tuple)
        self.setpoint_driver.lla = target_lla

        self.hover_pos = target_lla

    def on_hover_timer(self, message):
        return "succeeded_hover"
