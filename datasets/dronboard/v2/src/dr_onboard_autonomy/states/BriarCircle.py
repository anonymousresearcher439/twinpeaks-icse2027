from droneresponse_mathtools import Lla
import rospy
from typing import Optional

from dr_onboard_autonomy.air_lease import make_buffer_air_tunnel_func, make_circle_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    convert_Lla_to_LlaDict,
    LlaDict,
    convert_tuple_to_LlaDict,
)
from dr_onboard_autonomy.states import AirLeaser
from dr_onboard_autonomy.states.components import Gimbal
from dr_onboard_autonomy.states.components.trajectory import CircleTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState


"""Example JSON:

{
    "name": "BriarCircle",
    "class": "BriarCircle",
    "args": {
        "center_position":{
            "latitude": 41.60675408809003, 
            "longitude": -86.356016043474,
            "relative_altitude": 20
        },
        "stare_position": {
            "latitude": 41.606879523876685, 
            "longitude": -86.35603187231904,
            "relative_altitude": 1.0
        },
        "sweep_angle": 15,
        "speed": 5.0
    },
    "transitions": [
        {
            "target": "BriarHover",
            "condition": "succeeded_circle"
        }
    ]
}

"""


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class BriarCircle(BaseState):
    def __init__(
        self,
        center_position: LlaDict,
        stare_position: LlaDict,
        sweep_angle: float = 360,
        speed: float = 2.0,
        **kwargs,
    ):
        """
        Fly in a circle while pointing the camera
        The drone's starting location is considered the starting point.
        The radius of the circle is calculated from the starting location and the center point

        Args:
            center_position has its altitude as AMSL
            stare_position has its alt as AMSL
            sweep_angle is in degrees
            speed is in meters per second
        """

        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_circle", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)

        super().__init__(trajectory_class=CircleTrajectory, **kwargs)

        self.center_position = BriarLla(center_position, is_amsl=True)
        self.stare_position = BriarLla(stare_position, is_amsl=True)
        self.sweep_angle = float(sweep_angle)
        self.speed = float(speed)
        self._pass_through_kwargs = kwargs

        self.gimbal_driver = Gimbal(self)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self._air_leaser: Optional[AirLeaser] = None


    def on_entry(self, userdata):
        circle_air_tunnel_func = make_circle_air_tunnel_func(
            center_pos=self.center_position.ellipsoid.lla
        )
        self._air_leaser = AirLeaser(
            tunnel_func=circle_air_tunnel_func,
            **self._pass_through_kwargs
        )
        outcome = self._air_leaser.execute(userdata=userdata)
        if outcome != "succeeded":
            return outcome

        self.trajectory.start()
        self.trajectory.fly_circle(self.center_position.amsl.lla, self.sweep_angle, self.speed)
        self.gimbal_driver.start()
        self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup


    def on_position_message(self, message):
        if not self.trajectory.is_done():
            return

        pos = message["data"]
        pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        pos = BriarLla(convert_Lla_to_LlaDict(pos), is_amsl=False)

        final_pos = self.trajectory.setpoint_driver.lla
        final_pos = BriarLla(convert_tuple_to_LlaDict(final_pos))

        distance = pos.ellipsoid.lla.distance(final_pos.ellipsoid.lla)
        if distance < 1.0:
            rospy.loginfo("BriarCircle - circle trajectory completed")
            return "succeeded_circle"

    
    def on_exit(self, outcome, userdata):
        rospy.loginfo("BriarCircle - asking for air lease buffer zone at trajectory end point")
        buffer_air_tunnel_func = make_buffer_air_tunnel_func()
        '''
        asking for a new air lease completes the last air lease
        need to ask for buffer zone around current position
        '''
        self._air_leaser = AirLeaser(
            tunnel_func=buffer_air_tunnel_func,
            **self._pass_through_kwargs
        )
        outcome = self._air_leaser.execute(userdata=userdata)
        if outcome != "succeeded":
            return outcome

