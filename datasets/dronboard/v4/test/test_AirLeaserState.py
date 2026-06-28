from dataclasses import asdict
import importlib
import json
import math
import queue
from typing import Callable, List, Tuple, Union
import unittest
from unittest.mock import patch

from droneresponse_mathtools import Lla
from dr_onboard_autonomy.airlease import messages

import dr_onboard_autonomy.air_lease_service as air_lease_service
import dr_onboard_autonomy.airlease as airlease

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.models.kinematics import LlaPosition
from dr_onboard_autonomy.states import AirLeaser

from . import mock_types

# We need to patch the module where the `AirLeaser` class is defined.
# But doing this is tricky.
#
# The AirLeaser class is defined in the module `src/dr_onboard_autonomy/states/AirLeaser.py`.
# However, the `src/dr_onboard_autonomy/states/__init__.py` file also defines a variable named `AirLeaser`,
# which shadows the `AirLeaser.py` module when we try to import it.
#
# Therefore, we can't use `@patch("dr_onboard_autonomy.states.AirLeaser")` to patch the module,
# as it would instead patch the `AirLeaser` variable defined in `__init__.py`.
#
# Similarly, `@patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "my_variable")` won't work,
# because `"dr_onboard_autonomy.states.AirLeaser"` is a variable, not a module, so it can't be used as a key in `sys.modules`.
#
# To correctly patch the `AirLeaser.py` module, we can use this trick:
# First need to get a reference to the module object. We can do that using:
AirLeaser_module = importlib.import_module(AirLeaser.__module__)
# This gives us a reference to the module where `AirLeaser` class is defined.
# Now we can use `@patch.object` to patch this module.
# For example: `@patch.object(AirLeaser_module, "some_variable")` will patch "some_variable" in the
# `AirLeaser.py` module.


# Some of our tests require us to control what messages are in the AirLeaser queue
# For example, we have a few tests where we put an air lease response message in
# the _front_ of the queue. BaseState uses an object with the MessageQueue protocol.
# so long as we replace the message_queue with an object that implements the MessageQueue
# protocol, we can do whatever we want.
# 
# Given all that background, let's implement a class that implements the MessageQueue protocol
# We will extend deque and add the two required methods
class SpecialQueue(queue.deque):
    def get(self):
        return self.popleft()
    
    def put(self, item):
        self.append(item)

class TestAirLeaserExternal(unittest.TestCase):

    # This test case tries to take a black box approach to testing the AirLeaser state.
    # It assumes that the AirLeaser state will send air lease requests at a certain frequency:
    FREQUENCY = 1 # 1 request per second
    # It assumes that the AirLeaser state will create RepeatTimer.
    # and this RepeatTimer will be used to trigger the sending of requests.
    # This RepeatTimer will have this name:
    TIMER_NAME = "airlease_timer"
    # This RepeatTimer will have this period:
    TIMER_PERIOD = 0.25 # so a new timer event will be sent every 0.25 seconds
    
    # what message sender provides us with air lease response messages?
    AIR_LEASE_MESSAGE_SENDER = "airlease_status"
    # during our test, what is the UAV's name? We need this to determine the MQTT topic
    UAV_NAME = "test_drone"
    # what  MQTT topic do we receive air lease responses on?
    AIR_LEASE_RESPONSE_MQTT_TOPIC = f"drone/{UAV_NAME}/airlease/status"

    # How long do we pause sending requests after we get a response that denies our request?
    DENIED_PAUSE_TIME = 2.0 # seconds

    @staticmethod
    def make_response_message(request_number: int, approved: bool, drone_id: str=None, deadlock=False) -> dict:
        if drone_id is None:
            drone_id = TestAirLeaserExternal.UAV_NAME
        return {
            "drone_id": drone_id,
            "request_number": request_number,
            "approved": approved,
            "deadlock": deadlock,
        }
    
    @staticmethod
    def make_timer_message() -> dict:
        msg_type = TestAirLeaserExternal.TIMER_NAME
        delta_t = TestAirLeaserExternal.TIMER_PERIOD
        return mock_types.mock_message(msg_type, delta_t)
    
    @staticmethod
    def time_ns_generator() -> Callable[[None], int]:
        """A generator that will return a new time_ns value every time it's called.
        """
        _time_ns = 1_733_241_244_656_000_000
        def time_ns():
            nonlocal _time_ns
            _time_ns += 10_000_000
            return _time_ns
        return time_ns
    
    RequestMessage = Union[messages.Request, messages.MultiRequest]
    @staticmethod
    def extract_air_lease_request_details(msg: RequestMessage) -> Tuple[List[int], RequestMessage]:
        """Given an air lease request, copy it but set the request number to None
        This helps us compare requests during testing.

        Often we need to make sure that the requests we send are the same except
        the request number is supposed to change every time.

        Example:
        ```
        request_numbers, request_msg = extract_air_lease_request_details(request)
        ```

        Now we can compare `request_msg` to another request message. But we can also
        check that the request numbers. 
        """
        def clean_request(request):
            request_number = request.request_number
            # copy the request data
            req_data = asdict(request)
            # set the request number to None
            req_data['request_number'] = None
            return request_number, messages.Request(**req_data)
        
        if isinstance(msg, messages.Request):
            req_num, request = clean_request(msg)
            return [req_num], request
        
        # get the request number
        request_numbers = [request.request_number for request in msg.requests]
        air_tunnel_requests = []
        for request in msg.requests:
            _, request = clean_request(request)
            air_tunnel_requests.append(request)
        return request_numbers, messages.MultiRequest(requests=air_tunnel_requests)

    def test_init_ordinary_case(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home: BriarLla = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        start = home.ellipsoid.lla.move_ned(10, 0, -10)
        end_ellipsoid = start.move_ned(0, -30, 0)
        end_ellipsoid = BriarLla.from_lla(end_ellipsoid, is_amsl=False).llaPosition

        tunnel_func = airlease.make_waypoint_multi_air_tunnel_func(end_ellipsoid)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        self.assertIsInstance(air_leaser, AirLeaser)

  
    
    def test_land_method(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        pos = home.ellipsoid.lla.move_ned(0, 0, -15).to_lla()
        land_tunnel_pos = pos.move_ned(0, 0, pos.alt).to_lla() # move to the point where our Ellipsoidal altitude is zero

        tunnel_func = airlease.make_waypoint_air_tunnel_func(LlaPosition.from_lla(land_tunnel_pos, is_amsl=False))
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        air_leaser.air_lease.send_land_message()
        # make sure mqtt.publish was called with the correct topic and payload
        air_leaser.air_lease.mqtt.publish.assert_called_once()

