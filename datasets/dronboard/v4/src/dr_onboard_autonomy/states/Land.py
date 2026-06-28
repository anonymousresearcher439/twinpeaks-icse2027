from droneresponse_mathtools import Lla
from queue import Queue
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
from dr_onboard_autonomy.states.components import AirLease
import rospy
from typing import Dict, Optional

from .BaseState import BaseState
from dr_onboard_autonomy.airlease import make_waypoint_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.models import FCUCopterMode, FCULandedStatus, FCUState, TimeStampedLlaPosition
from dr_onboard_autonomy.states import AirLeaser, ReadMessagesAirborne

from dr_onboard_autonomy.state_factory import register_state


@register_state(default_transitions={
    "error": "failure",
    "failsafe": "Failsafe",
    "rtl": "Rtl" 
})
class Land(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_land", "error", "failsafe", "rtl"]
        super().__init__(**kwargs)

        # We want to use the LAND mode to land the drone
        # so we need to prevent this mode from triggering the RC Failsafe.
        self.rc_failsafe.remove_mode(FCUCopterMode.LAND)
        
        self._pass_through_kwargs = kwargs
        self._pass_through_kwargs['name'] = self.name
        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self._on_state_message)

        self.air_lease = AirLease(self, cleanup_messages=True)
        self._llaPosition: LlaPosition = None
        self._air_leaser: Optional[AirLeaser] = None
        self._current_position: Optional[Lla] = None
        self._pos_return_channel = Queue()

        self._position_reader: ReadMessagesAirborne = ReadMessagesAirborne(
            message_names=["position"],
            **kwargs
        )

        self.handlers.add_handler("position", self._on_position_message)


    def on_entry(self, userdata):
        outcome = self.execute_substate(self._position_reader, None)
        rospy.loginfo(f"Land - current position retrieval outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        sensor_data = self._position_reader.get()
        current_pos: BriarLla = BriarLla.from_ros(sensor_data["position"])
        '''
        ask for air tunnel from current altitude down to zero
        '''
        final_pos: Lla = current_pos.ellipsoid.lla.move_ned(0, 0, current_pos.ellipsoid.lla.alt)
        waypoint_air_tunnel_func = make_waypoint_air_tunnel_func(
            destination=LlaPosition(
                final_pos.latitude,
                final_pos.longitude,
                final_pos.altitude,
                datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
            )
        )

        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            **self._pass_through_kwargs
        )
        outcome = self.execute_substate(self._air_leaser, userdata)
        rospy.loginfo(f"Land - air lease outcome: {outcome}")
        if outcome != "succeeded":
            return outcome

        # This puts the drone in land mode...
        is_mode_change_success= self.drone.set_fcu_mode(FCUCopterMode.LAND)
        rospy.loginfo(f"LAND mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"


    def _on_position_message(self, message: Dict):
        pos: TimeStampedLlaPosition = message['data']
        self._llaPosition = pos
        self._current_position = Lla(pos.latitude, pos.longitude, pos.altitude)


    def _on_state_message(self, message):
        # if drone reports landed_state == LANDED_STATE_ON_GROUND
        # http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
        state: FCUState = message["data"]
        if state.landed_status == FCULandedStatus.ON_GROUND:
            # TODO Should we send the hover message????
            if self._current_position is not None:
                self.air_lease.send_hover_message(self._llaPosition) # TODO delete this line
            else:
                rospy.logwarn("Land - communicate_route_complete without providing position")
                self.air_lease.send_hover_message(self._llaPosition) # TODO delete this line
            return "succeeded_land"
