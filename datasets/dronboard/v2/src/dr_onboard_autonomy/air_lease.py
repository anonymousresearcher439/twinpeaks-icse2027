import math
from dataclasses import asdict
from typing import (
    Callable,
    List,
    NamedTuple,
    Tuple,
    Optional,
    Union,
)

from droneresponse_mathtools import Lla, Pvector
import rospy

from dr_onboard_autonomy.air_lease_requests import (
    Cleanup,
    HoverRequest,
    Land,
    MultiRequest,
    Request
)
from dr_onboard_autonomy.mqtt_client import MQTTClient


class AirTunnel(NamedTuple):
    """An air tunnel's start position, end position and radius.

    All altitudes values are in meters above the ellipsoid. The radius is in meters
    """
    start_pos: Lla
    end_pos: Lla
    radius: float


AirTunnel.start_pos.__doc__ = "The start position of the tunnel. Altitude is in meters above the ellipsoid"
AirTunnel.end_pos.__doc__ = "The end position of the tunnel. Altitude is in meters above the ellipsoid"
AirTunnel.radius.__doc__ = "The radius of the tunnel in meters"

"""The position passed to TunnelFunc must have its altitude in meters above the ellipsoid"""
TunnelFunc = Callable[[Lla], AirTunnel]
TunnelFunc.__doc__ = "A function that takes the current position and returns the air tunnel parameters"
TunnelFunc.__doc__ += " The current position has it's altitude in meters above the ellipsoid"


# TODO research a better value for this
# this is the radius of the air tunnel
air_tunnel_radius = 5.0


def make_circle_air_tunnel_func(center_pos: Lla) -> TunnelFunc:
    """Generates a function that will provide air tunnel parameters for circular trajectories

    Args:
        center_pos: Alt in meters above the ellipsoid
    """
    def circle_air_tunnel_func(current_pos: Lla) -> AirTunnel:
        n,e,d = current_pos.distance_ned(center_pos)
        radius = math.sqrt(n**2 + e**2)
        buffer = air_tunnel_radius
        # move horizontally to the center of the circle
        air_tunnel_center = current_pos.move_ned(n, e, 0)
        # drop below the center of the cirle to find the start position
        start_pos = air_tunnel_center.move_ned(0, 0, -buffer)
        # move up from the center of the circle to find the end position
        end_pos = air_tunnel_center.move_ned(0, 0, +buffer)
        return AirTunnel(start_pos, end_pos, radius + buffer)

    return circle_air_tunnel_func


def make_waypoint_air_tunnel_func(end_pos: Lla) -> TunnelFunc:
    """Generates a function that will provide air tunnel parameters for waypoint (linear)
    trajectories

    Args:
        end_pos: Alt in meters above the ellipsoid
    """
    def find_waypoint_air_tunnel(current_pos: Lla)-> AirTunnel:
        wp_start_pos = current_pos
        wp_end_pos = end_pos
        return AirTunnel(wp_start_pos, wp_end_pos, air_tunnel_radius)

    return find_waypoint_air_tunnel


def make_waypoint_multi_air_tunnel_func(end_pos: Lla) -> TunnelFunc:
    """Generates a function that will provide a sequence of air tunnel parameters for waypoint
    (linear) trajectories

    Args:
        end_pos: end position of the total trajectory. Alt in meters above the ellipsoid
    """
    '''
    TODO - make the air_tunnel_length smarter - dynamic based on velocity, max accel, and max jerk
    '''
    air_tunnel_length = 8.0
    def waypoint_multi_air_tunnel_func(current_pos: Lla)-> List[AirTunnel]:
        current_xyz = current_pos.to_pvector()
        end_xyz = end_pos.to_pvector()

        start_to_end = end_xyz.xyz - current_xyz.xyz
        magnitude = math.sqrt(start_to_end[0] ** 2 + start_to_end[1] ** 2 + start_to_end[2] ** 2)
        unit_start_to_end = (
            start_to_end[0] / magnitude,
            start_to_end[1] / magnitude,
            start_to_end[2] / magnitude
        )

        num_tunnels = magnitude / air_tunnel_length
        num_full_length_tunnels = math.floor(num_tunnels)

        tunnels = []
        '''
        create full length tunnels
        '''
        incremental_start_lla = current_pos
        for i in range(num_full_length_tunnels):
            incremental_movement = Pvector(
                *tuple([comp * (air_tunnel_length * (i + 1)) for comp in unit_start_to_end])
            )
            incremental_end_lla = Pvector(*(current_xyz.xyz + incremental_movement.xyz)).to_lla()
            tunnels.append(
                AirTunnel(
                    start_pos=incremental_start_lla,
                    end_pos=incremental_end_lla,
                    radius=air_tunnel_radius
                )
            )
            incremental_start_lla = incremental_end_lla

        '''
        case where only single tunnel shorter than full length tunnel
        '''
        if num_full_length_tunnels == 0:
            tunnels.append(
                AirTunnel(
                    start_pos=current_pos,
                    end_pos=end_pos,
                    radius=air_tunnel_radius
                )
            )

            return tunnels

        '''
        create final fractional tunnel
        '''
        tunnels.append(
            AirTunnel(
                start_pos=incremental_start_lla,
                end_pos=end_pos,
                radius=air_tunnel_radius
            )
        )

        return tunnels

    return waypoint_multi_air_tunnel_func


def make_buffer_air_tunnel_func() -> TunnelFunc:
    """Generates a function that will provide air tunnel for buffer around current position
    """
    def circle_air_tunnel_func(current_pos: Lla) -> AirTunnel:
        buffer = 5.0
        # tunnel with half buffer below and half above current position
        start_pos = current_pos.move_ned(0, 0, -buffer / 2)
        end_pos = current_pos.move_ned(0, 0, +buffer / 2)
        return AirTunnel(start_pos, end_pos, air_tunnel_radius)

    return circle_air_tunnel_func


class AirLeaseService:
    def __init__(self, uav_id: str, mqtt_client: MQTTClient):
        self.uav_id = uav_id
        self.mqtt = mqtt_client
        self._request_count = 0
        self._last_request = None
    
    def generate_final_request(self, request: Union[Request, MultiRequest]) -> Union[Request, MultiRequest]:
        """Create a new request with the correct `drone_id` and `request_number`. This method is used to create a request object that's ready to be sent to the air leasing service.

        This method takes an existing request, copies all its data, and generates a new request. The new request will have the same data as the input request, but with the `drone_id` and `request_number` fields determined by this object. These two fields combine to create the overall request ID.

        Note: The input request's `drone_id` and `request_number` fields are ignored. 

        This method can process the following types of requests:
            Request
            MultiRequest

        Note: Request and MultiRequest objects are immutable. That's why we have to return a new instance instead of updating the existing one.

        The return type will match the input type. For example, if you pass in a Request, you will get a Request back. If you pass in a MultiRequest, you will get a MultiRequest back.
        """
        if isinstance(request, MultiRequest):
            updated_requests = []
            for req in request.requests:
                updated_requests.append(self._build_one_request_internal(req))
            return MultiRequest(requests=updated_requests)
        elif isinstance(request, Request):
            #  true if isinstance(request, Request) or isinstance(request, HoverRequest)
            # make sure we have hte fields we need
            return self._build_one_request_internal(request)
        else:
            raise ValueError(f"Invalid request type: {type(request)}")
    
    def _build_one_request_internal(self, request) -> Request:
        """Create a new request with the `drone_id` and `request_number` fields filled in.

        All other data from the existing request will be copied over to the new one.

        We return a new one because Request is frozen and we can't update in place.
        """
        # we need to build a new request because we these instances are frozen
        self._request_count += 1
        data = asdict(request)
        data["request_number"] = self._request_count
        data["drone_id"] = self.uav_id
        return Request(**data)


    def send_request(self, msg: Union[Request, MultiRequest]):
        """Send a request to the air leasing service
        The msg must have the uav_id and request_number set using the prepare_request method

        Do not change the msg after calling this method
        """

        self.mqtt.publish(topic="airlease/request", data=asdict(msg), qos=0)

        # record the last request sent so that we can send a done message later
        if isinstance(msg, MultiRequest):
            self._last_request = msg.requests[-1]
        else:
            self._last_request = msg


    def send_done(self, position: Optional[Tuple[float, float, float]] = None):
        """Only should be called to complete previous request
        No response will be provided from the air leasing service - assume approved

        Args:
            position: in LLA where altitude is specified as meters above the ellipsoid
        """
        if self._last_request is None:
            rospy.logwarn("AirLeaseService - send_done ignored because no previous request sent")
        else:
            num = self._last_request.request_number
            if position is None:
                position = self._last_request.end_position 
            msg = HoverRequest(drone_id=self.uav_id, request_number=num, current_position=position)
            rospy.loginfo(f"AirLeaseService - send_done publishing 'hover' for {self.uav_id}")
            self.mqtt.publish(topic="airlease/hover", data=asdict(msg))


    def land(self):
        """Air lease service assumes drone at end position of last request
        """
        msg = Land(self.uav_id, self._request_count)
        self.mqtt.publish(topic="airlease/land", data=asdict(msg))


    def send_cleanup(self, position: Tuple[float, float, float]):
        """Send throughout flying state to update the air leasing service on the drone's current
        position. The drone's current position is used to remove completed air tunnels.

        Args:
            position: in LLA where altitude is specified as meters above the ellipsoid
        """
        msg = Cleanup(self.uav_id, position)
        self.mqtt.publish(topic="airlease/cleanup", data=asdict(msg))