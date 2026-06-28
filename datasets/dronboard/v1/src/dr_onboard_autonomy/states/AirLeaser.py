import json
import time
from typing import (
    Optional,
    Set
)
import rospy

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.air_lease_requests import MultiRequest as MultiTunnelRequest
from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest
from dr_onboard_autonomy.air_lease import AirTunnel, TunnelFunc
from dr_onboard_autonomy.briar_helpers import briarlla_from_ros_position
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.BaseState import BaseState
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory


class AirLeaser(BaseState):
    """Extends BaseState and asks for an air lease until one is granted

    Args:
        tunnel_func: Generates air lease Request args - provided from closures in 
            dr_onboard_autonomy.air_lease
            Not required with land set to True
        hold_position: should the current position of the drone be held via setpoint commands 
            (default=True)
            Not required for Takeoff
        unwanted_outcomes: this is a set of strings. These are outcomes to exclude from the outcome
            argument passed to smach.State. For example, the Takeoff state doesn't want the
            "human_control" outcome to be possible before lift-off. By default this is the empty
            set. Most of the time you can use the default.
    """
    def __init__(
        self,
        tunnel_func: Optional[TunnelFunc]=None,
        hold_position: bool=True,
        unwanted_outcomes: Set[str]=set(),
        **kwargs
    ):
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()

        for other_outcome in ["succeeded_airlease", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        
        outcome_set = outcome_set - unwanted_outcomes
        kwargs["outcomes"] = list(outcome_set)
        super().__init__(trajectory_class=WaypointTrajectory, **kwargs)

        self.tunnel_func: TunnelFunc = tunnel_func
        self.hold_position = hold_position
        
        self._trajectory_started: bool = False
        self._current_position: Optional[Lla] = None
        self._time_of_response: Optional[int] = None
        self._request_initial_airlease: bool = True

        self.message_senders.add(self.reusable_message_senders.find("airlease_status"))
        self.message_senders.add(RepeatTimer("airlease_timer", 1))

        self.handlers.add_handler("airlease_timer", self._on_timer)
        self.handlers.add_handler("airlease_status", self._on_airlease_status_message)
        self.handlers.add_handler("position", self._on_position_message)


    def _check_current_position(self) -> bool:
        if self._current_position is not None:
            return True

        rospy.logwarn("AirLeaser - air lease not requested due to lack of current position")
        return False


    def _on_airlease_status_message(self, message: dict):
        data_payload = json.loads(message["data"].payload)
        if data_payload["approved"]:
            rospy.loginfo("AirLeaser - air lease approved")
            if self._trajectory_started:
                self.trajectory.stop()
            return "succeeded"
        rospy.loginfo("AirLeaser - air lease denied")
        self._time_of_response = time.monotonic()


    def _on_timer(self, _):
        if self._time_of_response is not None:
            if time.monotonic() - self._time_of_response > 2:
                self._time_of_response = None
                self._ask_for_airlease()


    def _on_position_message(self, message: dict):
        pos = message['data']
        current_position = briarlla_from_ros_position(pos)
        self._current_position = current_position.ellipsoid.lla

        '''
        only request airlease once from position message handler
        '''
        if self._request_initial_airlease:
            self._ask_for_airlease()

        if self.hold_position and not self._trajectory_started:
            self.trajectory.start()
            self.trajectory.fly_to_waypoint(
                current_position.amsl.lla,
                3.0
            )
            self._trajectory_started = True


    def _ask_for_airlease(self):
        """Handles requesting air leases that require a response from the air lease service

        drone_id and request_number will be overridden by send_request and send_multi_request
        """
        if self._check_current_position():
            tunnel_response = self.tunnel_func(self._current_position)
            '''
            multi-tunnel functions return a list of AirTunnel objects
            '''
            if isinstance(tunnel_response[0], AirTunnel):
                tunnel_requests = []
                for air_tunnel in tunnel_response:
                    start_pos, end_pos, radius = air_tunnel
                    tunnel_requests.append(
                        TunnelRequest(
                            drone_id=self.drone.uav_name,
                            start_position=[start_pos.lat, start_pos.lon, start_pos.alt],
                            end_position=[end_pos.lat, end_pos.lon, end_pos.alt],
                            radius=radius,
                            request_number=0
                        )
                    )

                self.air_lease_service.send_multi_request(
                    requests=MultiTunnelRequest(requests=tunnel_requests)
                )
                rospy.loginfo("AirLeaser - Sent multi request to air lease service")
                self._request_initial_airlease = False
                return

            start_pos, end_pos, radius = tunnel_response
            tunnel_request = TunnelRequest(
                drone_id=self.drone.uav_name,
                start_position=[start_pos.lat, start_pos.lon, start_pos.alt],
                end_position=[end_pos.lat, end_pos.lon, end_pos.alt],
                radius=radius,
                request_number=0
            )

            self.air_lease_service.send_request(request=tunnel_request)
            rospy.loginfo("AirLeaser - Sent request to air lease service")
            self._request_initial_airlease = False
            return


    def communicate_route_complete(self, current_position: Optional[Lla]=None):
        """Completes last air lease

        Doesn't use AirLeaser._current_position as it is likely to be stale by the time
        this function is called
        """
        if current_position is not None:
            self.air_lease_service.send_done(
                position=[
                    current_position.lat,
                    current_position.lon,
                    current_position.alt
                ]
            )
        else:
            self.air_lease_service.send_done()


    def land(self):
        """Air leasing service does not provide a reply for land requests, so request and move on
        """
        self.air_lease_service.land()

        return "succeeded"