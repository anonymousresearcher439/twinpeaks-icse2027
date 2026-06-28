

from droneresponse_mathtools import Lla
from typing import Optional

from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, convert_tuple_to_LlaDict
from dr_onboard_autonomy.states import AirLeaser
from dr_onboard_autonomy.states.components import Gimbal
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory

from .BaseState import BaseState





DEFAULT_STARE_POS = (0.,0.,0.)
DEFAULT_STARE_POS = convert_tuple_to_LlaDict(DEFAULT_STARE_POS)


class BriarWaypoint(BaseState):
    def __init__(self, data=None, waypoint:LlaDict=None, stare_position:LlaDict=None, speed:float=2.0, **kwargs):
        """
        Fly to the specified waypoint taking the most direct path.

        Args:
            waypoint: the final destination. Alt is AMSL
            stare_position: where the camera should look. Alt is AMSL
            speed: how fast to fly in meters per second
        """
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)

        self.waypoint = BriarLla(waypoint)
        self.speed=float(speed)
        self.data = data
        self._pass_through_kwargs = kwargs

        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        if stare_position is not None:
            self.stare_position: BriarLla = BriarLla(stare_position)
            self.gimbal_driver = Gimbal(self)
        else:
            self.stare_position = None

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self._air_leaser: Optional[AirLeaser] = None
        
        self.debounce_logger = DebounceLogger()


    def on_entry(self, userdata):
        waypoint_air_tunnel_func = make_waypoint_multi_air_tunnel_func(
            end_pos=self.waypoint.ellipsoid.lla
        )
        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            **self._pass_through_kwargs
        )
        outcome = self._air_leaser.execute(userdata=userdata)
        if outcome != "succeeded":
            return outcome

        self.trajectory.start()
        self.trajectory.fly_to_waypoint(self.waypoint.amsl.lla, self.speed)

        if self.stare_position:
            self.gimbal_driver.start()
            self.gimbal_driver.stare_position = self.stare_position.ellipsoid.tup
    
    
    def on_position_message(self, message):
        pos = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        distance = current_pos.distance(self.waypoint.ellipsoid.lla)
        self.debounce_logger.info(f"BriarWaypoint: Current Position: {current_pos}", 1.0, 1)
        self.debounce_logger.info(f"BriarWaypoint: Distance to waypoint: {distance}", 1.0, 2)

        if distance < 1.5 and self.trajectory.is_done():
            self._air_leaser.communicate_route_complete(
                current_position=current_pos
            )
            return "succeeded_waypoints"
        
        if distance < 1.5:
            self.debounce_logger.info("Waypoint reached, but trajectory is not done yet.", 0.5)
        if self.trajectory.is_done():
            self.debounce_logger.info("Trajectory is done, but waypoint is too far away.", 0.5)


    def clean_up_air_lease(self, message):
        """TODO - Runs on a timer to send current position to the air lease service cleanup channel
        """
        pass

