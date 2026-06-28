#!/usr/bin/env python3
'''
TODO - Potentially replace with a package built from the msg.py file in the airlease service repo to 
automatically keep in sync
'''
from dataclasses import dataclass, asdict
from typing import List, Tuple
import json

'''
Classes to send messages for Air Leasing requests
'''

@dataclass(frozen=True)
class Request:
    '''
    Air leasing request. The drone sends this message to request permission to
    fly in some air space.

    The drone can send this message in 3 cases:
    1. When the drone has no routes and wants to make a new request.
    1. When the drone completes its route, and does not wish to hover but request a new route.
    2. When the drone wishes to update its current route to a new destination.

    The parameter, "request_number', should only increment when a route request is made, not when a
    hover or land request is made.

    Parameters:
        drone_id: str The name of the drone

        start_position: Tuple[float, float, float] latitude, longitude, altitude
            where the altitude is specified as meters above the ellipsoid

        end_position: Tuple[float, float, float] latitude, longitude, altitude
            where the altitude is specified as meters above the ellipsoid

        radius: Float meters

        request_number: Int a unique number per request

    Examples:
    1. Create a request:
    import msg
    r = Request("Red", [41.7148876122, -86.241774592, 185.65],
    [41.71489661544605, -86.24176257594874, 188.65],
    [41.714977644221385, -86.24165443184441, 215.65], 5, 0)
    r.dumps()
    2. Load a request from a json string:
    json_msg = '{"drone_id": "Red", "start_position": [41.7148876122, -86.241774592, 185.65],
    "start_position": [41.71489661544605, -86.24176257594874, 188.65],
    "end_position": [41.714977644221385, -86.24165443184441, 215.65], "radius": 5,
    "request_number": 0}'
    Request.loads(json_msg)
    '''

    drone_id: str
    start_position: Tuple[float, float, float]
    end_position: Tuple[float, float, float]
    radius: float
    request_number: int
    current_lease: int

    def dumps(self):
        out = asdict(self)
        return json.dumps(out)
    
    #accessible outside scope 
    @staticmethod
    def loads(json_str):
        in_dict = json.loads(json_str)
        return Request(**in_dict)


@dataclass(frozen=True)
class HoverRequest():
    '''
    Hover request. The drone sends a hover message when it reaches its end point, and needs its
    current airspace restricted as it waits/hovers in the air until it requests another route.
    The request number should match the previous request number sent and the drone should have
    completed its route.

    Parameters:
        drone_id: str The name of the drone
        request_number: int a unique number - matches the drone's most recent request (used to map
        to change the current air tunnel)
        current_position: Tuple[float, float, float] latitude, longitude, altitude where the
        altitude is specified as meters above the ellipsoid.
    '''

    drone_id: str
    request_number: int
    current_position: Tuple[float, float, float]

    def dumps(self):
        out = asdict(self)
        return json.dumps(out)

    #accessible outside scope
    @staticmethod
    def loads(json_str):
        in_dict = json.loads(json_str)
        return HoverRequest(**in_dict)


@dataclass(frozen=True)
class Land():
    '''
    The drone sends this message to when it reaches it is done requesting, or lands.
    The dictionary will remove all of the routes associated with the drone id from the dictionary.
    The request number should be sent for the dictionary to remove the most recent route -
    indicating a successful landing

    Parameters:
        drone_id: str The name of the drone
        request_number: int a unique number - the drones most recent request
    '''
    drone_id: str
    request_number: int

    def dumps(self):
        out = asdict(self)
        return json.dumps(out)
    
    @staticmethod
    def loads(json_str):
        in_dict = json.loads(json_str)
        return Land(**in_dict)


@dataclass(frozen=True)
class MultiRequest():
    """
    Same data as a request, but allows mutliple leases to be requested in one message. If any of the
    multiple requests is denied, all the requests will be denied. 
    """
    requests: List[Request]

    def dumps(self):
        out = asdict(self)
        return json.dumps(out)

    @staticmethod
    def loads(json_str):
        in_dict = json.loads(json_str)
        requests_from_dict = [Request(**request_dict) for request_dict in in_dict["requests"]]
        return MultiRequest(requests=requests_from_dict)


@dataclass(frozen=True)
class Cleanup():
    """
    Removes old leases based on the drone's current position and the associated lease the drone is 
    within. Any leases with request numbers lower than the request number of the drone's currently
    associated lease will be dropped.

    Args:
        drone_id: the name of the drone
        position: in LLA where altitude is specified as meters above the ellipsoid
    """
    drone_id: str
    position: Tuple[float, float, float]

    def dumps(self):
        out = asdict(self)
        return json.dumps(out)

    @staticmethod
    def loads(json_str):
        in_dict = json.loads(json_str)
        return Cleanup(**in_dict)
