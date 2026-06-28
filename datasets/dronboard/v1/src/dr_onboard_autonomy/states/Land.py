from droneresponse_mathtools import Lla
from queue import Queue
import rospy
from typing import Optional

from .BaseState import BaseState
from dr_onboard_autonomy.air_lease import make_waypoint_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import AirLeaser, ReadPosition


class Land(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_land", "error", "human_control", "rtl"]
        super().__init__(**kwargs)
        self._pass_through_kwargs = kwargs
        self.message_senders.add(self.reusable_message_senders.find("extended_state"))
        self.handlers.add_handler("extended_state", self._on_extended_state_message)

        self._air_leaser: Optional[AirLeaser] = None
        self._current_position: Optional[Lla] = None
        self._pos_return_channel = Queue()
        self._position_reader = ReadPosition(return_channel=self._pos_return_channel, **kwargs)

        self.handlers.add_handler("position", self._on_position_message)


    def on_entry(self, userdata):
        outcome = self._position_reader.execute(None)
        rospy.loginfo(f"Land - current position retrieval outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        current_pos: BriarLla = self._pos_return_channel.get()
        '''
        ask for air tunnel from current altitude down to zero
        '''
        final_pos: Lla = current_pos.ellipsoid.lla.move_ned(0, 0, current_pos.ellipsoid.lla.alt)
        waypoint_air_tunnel_func = make_waypoint_air_tunnel_func(end_pos=final_pos)

        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            **self._pass_through_kwargs
        )
        outcome = self._air_leaser.execute(userdata=userdata)
        rospy.loginfo(f"Land - air lease outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        
        # This puts the drone in land mode...
        is_mode_change_success = self.drone.land()
        rospy.loginfo(f"LAND mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"


    def _on_position_message(self, message: dict):
        pos = message['data']
        self._current_position = Lla(pos.latitude, pos.longitude, pos.altitude)


    def _on_extended_state_message(self, message):
        if "extended_state" not in self.message_data:
            return

        data = self.message_data
        # if drone reports landed_state == LANDED_STATE_ON_GROUND
        # http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
        if data["extended_state"].landed_state == 1:
            if self._current_position is not None:
                self._air_leaser.communicate_route_complete(self._current_position)
            else:
                rospy.logwarn("Land - communicate_route_complete without providing position")
                self._air_leaser.communicate_route_complete()
            return "succeeded_land"
