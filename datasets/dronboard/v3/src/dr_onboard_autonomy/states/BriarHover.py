from typing import Optional

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    LlaDict
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, UpdateStarePosition
from dr_onboard_autonomy.states.components.trajectory import YawTrajectory, HoldingTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState


"""Example JSON:

{
    "name": "BriarHover",
    "class": "BriarHover",
    "args": {
        "hover_time": 5.0,
        "stare_position": {
            "latitude": 41.606879523876685, 
            "longitude": -86.35603187231904,
            "relative_altitude": 1.0
        }
    },
    "transitions": [
        {
            "target": "BriarTravel",
            "condition": "succeeded_hover"
        }
    ]
}

"""


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class BriarHover(BaseState):
    def __init__(self, hover_time:float=10, stare_position:Optional[LlaDict]=None, **kwargs):
        """
        Hovers the drone while pointing the camera.

        Args:
            hover_time: float in seconds. How long the drone will hover.
            stare_position: where the camera looks. Alt is AMSL.
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_hover"], STANDARD_TRANSITIONS_FLYING.keys())


        TrajectoryClass = YawTrajectory
        if stare_position is None:
             TrajectoryClass = HoldingTrajectory

        super().__init__(trajectory_class=TrajectoryClass, **kwargs)

        self.hover_time = hover_time
        self.stare_position = None

        if stare_position is not None:
            stare_position: BriarLla = BriarLla(stare_position, is_amsl=True)
            self.stare_position = stare_position.ellipsoid.tup
            self.gimbal_driver = Gimbal(self)
            self.stare_position_updater = UpdateStarePosition(self, self.gimbal_driver)

        self.message_senders.add(RepeatTimer("hover_timer", hover_time))
        self.handlers.add_handler("hover_timer", self.on_hover_timer)

    def on_entry(self, userdata):
        self.trajectory.start()
        if self.stare_position is not None:
            self.gimbal_driver.start()
            self.gimbal_driver.stare_position = self.stare_position
            # need to set the setpoint lla so we don't drift using velocity=zero 
            current_pos = self.drone.data.location.get_position()
            current_pos = current_pos.to_amsl()
            self.trajectory.setpoint_driver.lla = (
                current_pos.latitude,
                current_pos.longitude,
                current_pos.altitude
            )

    def on_hover_timer(self, message):
        return "succeeded_hover"
