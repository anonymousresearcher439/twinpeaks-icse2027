import math
import time
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
import numpy as np
import rospy

from dr_onboard_autonomy.airlease.messages import (
    Cleanup,
    HoverRequest,
    Land,
    MultiRequest,
    Request
)
from dr_onboard_autonomy.mqtt_client import MQTTClient


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
        self._request_count = time.time_ns() // 1_000_000 # milliseconds
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