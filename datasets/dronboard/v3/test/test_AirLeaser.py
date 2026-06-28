from dataclasses import asdict
import importlib
import json
import math
import queue
from typing import Callable, List, Tuple, Union
import unittest
from unittest.mock import patch

from droneresponse_mathtools import Lla
from dr_onboard_autonomy import air_lease_requests

import dr_onboard_autonomy.air_lease as air_lease

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import AirLeaser
from dr_onboard_autonomy.states.AirLeaser import _State as air_leaser_interal_state

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
    
    RequestMessage = Union[air_lease_requests.Request, air_lease_requests.MultiRequest]
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
            return request_number, air_lease_requests.Request(**req_data)
        
        if isinstance(msg, air_lease_requests.Request):
            req_num, request = clean_request(msg)
            return [req_num], request
        
        # get the request number
        request_numbers = [request.request_number for request in msg.requests]
        air_tunnel_requests = []
        for request in msg.requests:
            _, request = clean_request(request)
            air_tunnel_requests.append(request)
        return request_numbers, air_lease_requests.MultiRequest(requests=air_tunnel_requests)

    def test_init_ordinary_case(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home: BriarLla = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        start = home.ellipsoid.lla.move_ned(10, 0, -10)
        end_ellipsoid = start.move_ned(0, -30, 0)

        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(end_ellipsoid)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        self.assertIsInstance(air_leaser, AirLeaser)
    
    def test_init_takeoff(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home: BriarLla = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        takeoff_waypoint = home.ellipsoid.lla.move_ned(0, 0, -10)
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(takeoff_waypoint)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=False,
            **kwargs
        )

        self.assertIsInstance(air_leaser, AirLeaser)
        # make sure we don't have a trajectory
        self.assertIsNone(air_leaser.trajectory, "we should not have a trajectory before takeoff")

    
    def test_init_land(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        pos = home.ellipsoid.lla.move_ned(0, 0, -15)
        land_tunnel_pos = pos.move_ned(0, 0, pos.alt) # move to the point where our Ellipsoidal altitude is zero

        tunnel_func = air_lease.make_waypoint_air_tunnel_func(land_tunnel_pos)
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        self.assertIsInstance(air_leaser, AirLeaser)

    @patch('dr_onboard_autonomy.air_lease.time.time_ns')
    @patch.object(AirLeaser_module.random, 'uniform')
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_send_requests(self, mock_ReadMessages, mock_ReadMessagesAirborne, mock_uniform, mock_time_ns):
        """Make sure that we send air lease requests

        The AirLease state should send air lease requests by calling air_leaser_service.send_request()

        It should call this method repeatedly until we get a response. And if we
        never get a response, then we should repeat this forever.

        This test wraps a real air_leaser_service instance in a mock.
        We verify that the send_request() method is called the correct number of times.
        We also verify that the send_request() method is called with the correct
        argument every time.

        We also mock the ReadMessagesAirborne class so that AirLease.on_entry()
        can read the current position of the drone.

        This test puts a bunch of timer messages in the AirLease state's message queue.
        Last it puts a shutdown message in the queue.

        Then it calls AirLease.execute() and verifies that the correct behavior occurs.

        We make sure that the correct number of requests are sent.
        We make sure that we used the correct positions in the request.
        We make sure that we used the corect ReadMessages class to get the current position.
        We make sure that we used the expected request numbers.

        """
        mock_uniform.return_value = 1.0
        mock_time_ns.side_effect = TestAirLeaserExternal.time_ns_generator()
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home: BriarLla = BriarLla.from_dict(kwargs["home"], is_amsl=True)

        start = home.ellipsoid.lla.move_ned(10, 0, -10)
        current_pos: BriarLla = BriarLla.from_lla(start, is_amsl=False) 
        end_ellipsoid = start.move_ned(0, -30, 0)

        # setup our `ReadMessagesClass`
        # AirLeaser will use this state internally to get the current position
        # we want two different instances of this so we can test that it uses the correct one
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(current_pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(current_pos)

        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(end_ellipsoid)

        # init an ordinary AirLeaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        # before we call execute, we want to replace the message_queue with 
        # `queue.Queue`. This is because the specialized message queue that 
        # `BaseState` normally uses will let some newer messages overwrite older
        # ones of the same type. I don't think this optimazation will impact us
        # given that we're putting timer messages in the queue. But we want our
        # tests to work if we change this behavior. Replacing the type of queue
        # also makes our tests easier to write and understand.
        air_leaser.message_queue = queue.Queue() 
        
        # let's add timer messages to the queue
        # we will send this many timer messages:
        number_of_timer_messages = 31
        # we will keep track of hte number of timer messages we send
        # so that we can calculate the total time we've simulated
        total_time = 0.0
        for _ in range(number_of_timer_messages):
            timer_msg = self.make_timer_message()
            delta_t = timer_msg['data']
            air_leaser.message_queue.put(timer_msg)
            total_time += delta_t
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())
        
        # now we can call execute
        outcome = air_leaser.execute(userdata=None)
        # we should have gotten an error outcome be cause we put a shutdown message in the queue
        self.assertEqual(outcome, "error") 

        # so we should have sent requests at our given FREQUENCY
        # therefore the number of requests we should have sent equals:
        expected_num_sent = math.floor(total_time * TestAirLeaserExternal.FREQUENCY) + 1
        # We need to add 1 because we send a message when we first enter the state

        # next make sure we sent the correct number of requests:
        air_lease_service = kwargs["air_lease_service"]
        self.assertEqual(air_lease_service.send_request.call_count, expected_num_sent) 

        # Next let's make sure we sent the correct message.
        # Specifically:
        #  - we need to make sure we sent the same message every time
        #  - we need to make sure the message we sent had the right values
        #
        # An AirLeaser object is supposed to build one request and send it over
        # and over again without changing it. It will keep sending the same request
        # until it gets a response. So we need to make sure that the exact same request
        # was sent every time.
        #
        # since our instance of air_lease_service is wrapped by a Mock, we can
        # use call_args_list to get the details about every call AirLeaser made:
        call_list = air_lease_service.send_request.call_args_list

        time_ns_gen = TestAirLeaserExternal.time_ns_generator()
        base_req_numbers = [(time_ns_gen() // 1_000_000) + 1 for _ in range(expected_num_sent)]

        # Now we can assert that every element in the list is equal to the first element.
        # Therefore, by the transitive property, every element is equal to every other element
        expected_arg = call_list[0].args[0] # from the 0th call to send_request get the first arg
        # we want to make sure all the request details match up except the request number
        # so our "expected arg" is the same as the first arg in the call list, but we will set
        # the request number to None. This way we can compare it
        print(expected_arg)
        _, expected_arg = TestAirLeaserExternal.extract_air_lease_request_details(expected_arg)

        for call, expected_req_num0 in zip(call_list, base_req_numbers):
            request = call.args[0]
            req_nums, request = TestAirLeaserExternal.extract_air_lease_request_details(request)
            self.assertEqual(request, expected_arg)
            # to check the request numbers we make sure the first one is the same as expected_req_num0
            # then we make sure each subsequent request number is 1 greater than the previous
            self.assertEqual(req_nums[0], expected_req_num0)
            for i in range(1, len(req_nums)):
                self.assertEqual(req_nums[i], req_nums[i-1] + 1)
        
        # besides making sure we send the same request, we need to make sure sure
        # the data in the request is correct. For context:
        #   air_lease_service.send_request()
        # takes a single argument, which is the request we want to send.

        # we should have a MultiRequest
        self.assertIsInstance(expected_arg, air_lease_requests.MultiRequest)

        # Since we have a MultiRequest, we will inspect the list of requests and
        # to sure the starting pos of the first AirTunnel is at the drone's
        # current position. We will also make sure that the far end of the last
        # AirTunnel is at the destination
        # get the list of AirTunnels
        requests = expected_arg.requests 
        
        # create an Lla for the actual start position
        actual_start_pos = requests[0].start_position
        actual_start_pos = Lla(*actual_start_pos)

        # create an Lla for the actual end position
        actual_end_pos = requests[-1].end_position
        actual_end_pos = Lla(*actual_end_pos)

        # the expected values are hard-coded above for the scenario we are testing
        # They're arbitrary. We just need to make sure they match
        expected_start_pos = start 
        expected_end_pos = end_ellipsoid

        # make sure the distance from actual_start_pos to the expected_start_pos is less than 0.01 meters
        # that's close enough to indicate that they are at the same position
        self.assertLessEqual(expected_start_pos.distance(actual_start_pos), 0.01)
        # do the same for the end position
        self.assertLessEqual(expected_end_pos.distance(actual_end_pos), 0.01)

        # make sure we used the right type to get the current position.
        # when we initialized the AirLeaser state, we set `is_flying` to True.
        # Therefore we should have used `ReadMessagesAirborne` to get our
        # position. Let's make sure that's true:
        mock_ReadMessagesAirborne.assert_called()
        mock_ReadMessagesAirborne.return_value.execute.assert_called()

        # Furthermore make sure we did _not_ use a ReadMessages instance
        # to read the current position.
        #
        # Technically, we shouldn't care if a ReadMessages instance was created.
        # We only care that it's execute() method was _not_ called.
        #
        # We need to do this because:
        # - a PreCondition for ReadMessages is that the drone is _not_ flying
        # - a PreCondition for ReadMessagesAirborne is that the drone _is_ flying
        
        # Make sure we did _not_ use a ReadMessages instance:
        mock_ReadMessages.return_value.execute.assert_not_called()
    
    @patch('dr_onboard_autonomy.air_lease.time.time_ns')
    @patch.object(AirLeaser_module.random, 'uniform')
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_send_requests_on_takeoff(self, mock_ReadMessages, mock_ReadMessagesAirborne, mock_uniform, mock_time_ns):
        """Make sure that we send air lease requests when we are taking off

        The AirLease state should send air lease requests by calling air_leaser_service.send_request()

        It should call this method repeatedly until we get a response. And if we
        never get a response, then we should repeat this forever.

        This test wraps a real air_leaser_service instance in a mock.
        We verify that the send_request() method is called the correct number of times.
        We also verify that the send_request() method is called with the correct
        argument every time.

        We also mock the ReadMessages class so that AirLease.on_entry()
        can read the current position of the drone.

        This test puts a bunch of timer messages in the AirLease state's message queue.
        Last it puts a shutdown message in the queue.

        Then it calls AirLease.execute() and verifies that the correct behavior occurs.

        We make sure that the correct number of requests are sent.
        We make sure that we used the correct positions in the request.
        We make sure that we used the corect ReadMessages class to get the current position.
        We make sure that we sent the same request every time.
        """
        mock_uniform.return_value = 1.0
        mock_time_ns.side_effect = TestAirLeaserExternal.time_ns_generator()

        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home: BriarLla = BriarLla.from_dict(kwargs["home"], is_amsl=True)

        start = home.ellipsoid.lla.move_ned(10, 0, -10)
        current_pos: BriarLla = BriarLla.from_lla(start, is_amsl=False) 
        end_ellipsoid = start.move_ned(0, -30, 0)

        # setup our `ReadMessagesClass`
        # AirLeaser will use this state internally to get the current position
        # we want two different instances of this so we can test that it uses the correct one
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(current_pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(current_pos)

        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(end_ellipsoid)

        # init an AirLeaser object for the takeoff case
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,

            # These two args match what the Takeoff state does:
            is_flying=False, 
            unwanted_outcomes={'failsafe', 'rtl'},

            **kwargs
        )

        # before we call execute, we want to replace the message_queue with 
        # `queue.Queue`. This is because the specialized message queue that 
        # `BaseState` normally uses will let some newer messages overwrite older
        # ones of the same type. I don't think this optimazation will impact us
        # given that we're putting timer messages in the queue. But we want our
        # tests to work if we change this behavior. Replacing the type of queue
        # also makes our tests easier to write and understand.
        air_leaser.message_queue = queue.Queue() 
        
        # let's add timer messages to the queue
        # we will send this many timer messages:
        number_of_timer_messages = 31
        # we will keep track of the number of timer messages we send
        # so that we can calculate the total time we've simulated
        total_time = 0.0
        for _ in range(number_of_timer_messages):
            timer_msg = self.make_timer_message()
            delta_t = timer_msg['data']
            air_leaser.message_queue.put(timer_msg)
            total_time += delta_t
        # add the shutdown message
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())
        
        # now we can call execute
        outcome = air_leaser.execute(userdata=None)
        # we should have gotten an error outcome be cause we put a shutdown message in the queue
        self.assertEqual(outcome, "error") 

        # How many requests should we have sent?
        # To calculate this, given FREQUENCY and total_time  
        # the number of requests sent equals:
        expected_num_sent = math.floor(total_time * TestAirLeaserExternal.FREQUENCY) + 1
        # We need to add 1 because we send a message when we first enter the state

        # make sure we sent the correct number of requests:
        air_lease_service = kwargs["air_lease_service"]
        self.assertEqual(air_lease_service.send_request.call_count, expected_num_sent) 

        # Next let's make sure we sent the correct message.
        # Specifically:
        #  - we need to make sure we sent the almost the same request message every time
        #  - this helps us sure we sent the correct air tunnel data every time
        #  - later we check that the first request message has correct data
        #
        # An AirLeaser object is supposed to build one request and send it over
        # and over again (with a new reqest_number). It will keep sending the same request
        # until it gets a response. So we need to make sure that the exact same air tunnels
        # were sent every time.
        #
        # since our instance of air_lease_service is wrapped by a Mock, we can
        # use call_args_list to get the details about every call AirLeaser made:
        call_args_list = air_lease_service.send_request.call_args_list
        # get the first request message we sent:
        req0 = call_args_list[0].args[0] 
        # to help us ensure the request numbers are correct we will check numbers
        # while we are making sure all the air tunnel details are the same every time
        # we will use this generator again to find the expected request numbers
        time_ns_gen = TestAirLeaserExternal.time_ns_generator()
        # the request numbers should be the current time in milliseconds
        # but we add 1 as an implementation detail. So here's the expected request_number
        # for the first air tunnel in each call to send_request
        expected_base_req_numbers = [(time_ns_gen() // 1_000_000) + 1 for _ in range(expected_num_sent)]

        # Now we can assert that every element in the list is equal to the first element.
        # first we need to seperate the air tunnel details from the request numbers
        _, req0 = TestAirLeaserExternal.extract_air_lease_request_details(req0)
        # req0 is nearly the same. it has the air tunnel details but the request numbers are all None

        for call, expected_req_num0 in zip(call_args_list, expected_base_req_numbers):
            actual_request = call.args[0]
            req_nums, actual_request = TestAirLeaserExternal.extract_air_lease_request_details(actual_request)
            self.assertEqual(actual_request, req0)
            # to check the request numbers are correct.
            # The 1st request number should be the same as expected_req_num0
            # and each subsequent request number should be 1 greater than the previous
            self.assertEqual(req_nums[0], expected_req_num0)
            for i in range(1, len(req_nums)):
                self.assertEqual(req_nums[i], req_nums[i-1] + 1)
        
        # besides making sure we send the correct request, we need to make sure sure
        # the data in the request is correct. For context:
        #   air_lease_service.send_request()
        # takes a single argument, which is the request we want to send.
        
        request_msg = req0 # this is the 1st request we sent

        # we should have a MultiRequest
        self.assertIsInstance(request_msg, air_lease.MultiRequest)

        # Since we have a MultiRequest, we will inspect the list of requests and
        # to sure the starting pos of the first AirTunnel is at the drone's
        # current position. We will also make sure that the far end of the last
        # AirTunnel is at the destination
        # get the list of AirTunnels
        requests = request_msg.requests 
        
        # create an Lla for the actual start position
        actual_start_pos = requests[0].start_position
        actual_start_pos = Lla(*actual_start_pos)

        # create an Lla for the actual end position
        actual_end_pos = requests[-1].end_position
        actual_end_pos = Lla(*actual_end_pos)

        # the expected values are hard-coded above for the scenario we are testing
        # They're arbitrary. We just need to make sure they match
        expected_start_pos = start 
        expected_end_pos = end_ellipsoid

        # make sure the distance from actual_start_pos to the expected_start_pos is less than 0.01 meters
        # that's close enough to indicate that they are at the same position
        self.assertLessEqual(expected_start_pos.distance(actual_start_pos), 0.01)
        # do the same for the end position
        self.assertLessEqual(expected_end_pos.distance(actual_end_pos), 0.01)

        # make sure we used the right type to get the current position.
        # when we initialized the AirLeaser state, we set `is_flying` to False.
        # Therefore we should have used `ReadMessages` to get our
        # position. Let's make sure that's true:
        mock_ReadMessages.assert_called()
        mock_ReadMessages.return_value.execute.assert_called()

        # Furthermore let's make sure we did _not_ use a mock_ReadMessagesAirborne
        # instance. We don't care if a ReadMessagesReadMessagesAirborne instance
        # was created. We only care that it's execute() method was _not_ called.

        # Make sure ReadMessagesAirborne.execute() was _not_ called
        mock_ReadMessagesAirborne.return_value.execute.assert_not_called()
    
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_respond_to_approved_requests(self, mock_ReadMessages, mock_ReadMessagesAirborne):
        """In this test we will send a multi-tunnel request, and provide a response that approves the request.

        The AirLease state should send an air lease request in it's on_entry 
        method by calling air_leaser_service.send_request(). So we will tap into
        this method and generate a response that approves the request. We will then
        put this response at the front of the message queue.  However, since queue.Queue
        Doesn't let you put an element at the front of the queue, we will 
        extend deque to make a new class that does this. We will then replace the
        message_queue with an instance of this new class.

        Before calling the execute() method, we will put a shutdown message in
        the message queue. To pass the test the AirLeaser state should respond to the
        "approved" message rather than the shutdown message.
        """

        # let's start by building our kwargs
        # we need this to create much of the data for our test
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        # we need to mock the ReadMessages classes
        # to do so we need a value for the drone's current position
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        current_pos = home.ellipsoid.lla.move_ned(0, 0, -10)
        current_pos = BriarLla.from_lla(current_pos, is_amsl=False)
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(current_pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(current_pos)

        message_queue = SpecialQueue()

        # next we need to define a function that will generate the approved response
        def send_approved_response(request):
            # we will put our response in this message queue
            nonlocal message_queue

            # Let's test that AirLeaser gave us the correct type of request
            # we're using a multi-request in this test
            self.assertIsInstance(request, air_lease.MultiRequest)

            # grab the request number because we need to reference it
            # in our response
            req_num = request.requests[0].request_number
            
            # next let's create a response that approves the request
            # since the structure of the response is part of the external
            # interface and we might change the structure of these messages in
            # #the future, we will use a method to create the response
            # data:
            msg_data = self.make_response_message(req_num, approved=True)
            assert isinstance(msg_data, dict)

            # now let's create a mock message from an MQTT Message sender
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC
            payload = json.dumps(msg_data)
            approved_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=999,
                qos=2,
            )

            # Last we put this response in the message queue
            # remember this is a deque and we want to put it at the front
            message_queue.appendleft(approved_response)

        # now we update our mock air_leaser_service instance so that it will
        # call our function when AirLeaser tries to send a request
        kwargs['air_lease_service'].send_request.side_effect = send_approved_response

        # Let's build our AirLeaser object
        # we will use a multi-request in this test
        dest = home.ellipsoid.lla.move_ned(10, 10, -10)
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        # very important: replace the message_queue with our special queue
        # we will put the approved response in this queue
        # so we need to make sure it's the one that AirLeaser uses
        air_leaser.message_queue = message_queue

        # we need to put a shutdown message in the message_queue
        # to prevent the AirLeaser state from blocking forever
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())

        outcome = air_leaser.execute(userdata=None)
        # make sure our test passed!
        self.assertEqual(outcome, "succeeded")
    
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_respond_to_approved_single_tunnel_requests(self, mock_ReadMessages, mock_ReadMessagesAirborne):
        """In this test we will send a single tunnel request, and provide a response that approves the request.

        The AirLease state should send an air lease request in it's on_entry 
        method by calling air_leaser_service.send_request(). So we will tap into
        this method and generate a response that approves the request. We will then
        put this response at the front of the message queue.  However, since queue.Queue
        Doesn't let you put an element at the front of the queue, we will 
        extend deque to make a new class that does this. We will then replace the
        message_queue with an instance of this new class.

        Before calling the execute() method, we will put a shutdown message in
        the message queue. To pass the test the AirLeaser state should respond to the
        "approved" message rather than the shutdown message.
        """

        # let's start by building our kwargs
        # we need this to create much of the data for our test
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        # we need to mock the ReadMessages classes
        # to do so we need a value for the drone's current position
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        current_pos = home.ellipsoid.lla.move_ned(0, 0, -10)
        current_pos = BriarLla.from_lla(current_pos, is_amsl=False)
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(current_pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(current_pos)

        message_queue = SpecialQueue()

        # next we need to define a function that will generate the approved response
        def send_approved_response(request):
            # we will put our response in this message queue
            nonlocal message_queue

            # Let's test that AirLeaser gave us the correct type of request
            # we're using a multi-request in this test
            self.assertIsInstance(request, air_lease.Request)

            # grab the request number because we need to reference it
            # in our response
            req_num = request.request_number
            
            # next let's create a response that approves the request
            # since the structure of the response is part of the external
            # interface and we might change the structure of these messages in
            # #the future, we will use a method to create the response
            # data:
            msg_data = self.make_response_message(req_num, approved=True)
            assert isinstance(msg_data, dict)

            # now let's create a mock message from an MQTT Message sender
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC
            payload = json.dumps(msg_data)
            approved_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=999,
                qos=2,
            )

            # Last we put this response in the message queue
            # remember this is a deque and we want to put it at the front
            message_queue.appendleft(approved_response)

        # now we update our mock air_leaser_service instance so that it will
        # call our function when AirLeaser tries to send a request
        kwargs['air_lease_service'].send_request.side_effect = send_approved_response

        # Let's build our AirLeaser object
        # we will use a single-tunnel request in this test
        dest = home.ellipsoid.lla.move_ned(10, 10, -10)
        tunnel_func = air_lease.make_waypoint_air_tunnel_func(dest)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        # very important: replace the message_queue with our special queue
        # we will put the approved response in this queue
        # so we need to make sure it's the one that AirLeaser uses
        air_leaser.message_queue = message_queue

        # we need to put a shutdown message in the message_queue
        # to prevent the AirLeaser state from blocking forever
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())

        outcome = air_leaser.execute(userdata=None)
        # make sure our test passed!
        self.assertEqual(outcome, "succeeded")
    
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_do_not_send_requests_when_mqtt_disconnected(self, mock_ReadMessages, mock_ReadMessagesAirborne):
        """In this test we will provide AirLeaser with timer messages. But we will
        disconnect the MQTT client before calling execute(). We will then verify
        that no requests are sent. Specifically we will make sure that
        air_lease_service.send_request() is not called.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # we need our test scenario to specify the drone's current position
        # and destination. 
        # first let's define the current position:
        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        # now for the destination:
        dest = home.ellipsoid.lla.move_ned(10, 10, -5)

        # time to mock the ReadMessages classes
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        # build the air tunnel function
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest)

        # change our MQTT client so that it appears disconnected
        mqtt = kwargs["mqtt_client"] # this is a mock instance
        mqtt.is_connected.return_value = False
        self.assertFalse(mqtt.is_connected())

        # build the air leaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )
        # replace the message_queue with a regular queue
        # we will put timer messages in this queue
        air_leaser.message_queue = queue.Queue()

        # How many timer messages should we send? Let's send 100
        for _ in range(100):
            msg = self.make_timer_message()
            air_leaser.message_queue.put(msg)
        # the goal is to make sure we don't send a single request since
        # the mqtt_client is disconnected. So adding all these
        # timer messages should be enough to make sure we don't send a request
        # Last add the shutdown message
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())
        # the shutdown message causes the execute method to return
        # otherwise we'd block forever.

        # time to run the test
        outcome = air_leaser.execute(None)
        self.assertEqual(outcome, "error")

        # now make sure we never called air_lease_service.send_request()
        air_lease_service = kwargs["air_lease_service"]
        # we should not have called send_request() at all
        air_lease_service.send_request.assert_not_called()
    
    @patch.object(AirLeaser_module.random, 'uniform')
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_resume_sending_requests_when_mqtt_reconnects(self, mock_ReadMessages, mock_ReadMessagesAirborne, mock_uniform):
        """In this test will make the MQTT client appear to disconnect for some
        time. Then we will make it appear like it reconnects
        
        We will make sure the system stops sending messages while we're disconnected
        but also that it resumes when MQTT reconnects
        
        We will tap into air_lease_service.send_request(). After N messages
        are sent, we will disconnect the MQTT client. We will then after
        the system checks the MQTT client's connection status a few times, we will
        reconnect the MQTT client. We will then make sure that the system resumes
        sending messages
        """
        mock_uniform.return_value = 1.0
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # we need our test scenario to specify the drone's current position
        # and destination. 
        # first let's define the current position:
        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        # now for the destination:
        dest = home.ellipsoid.lla.move_ned(10, 10, -5)

        # time to mock the ReadMessages classes
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        # build the air tunnel function this time well make a single tunnel
        tunnel_func = air_lease.make_waypoint_air_tunnel_func(dest)

        # tap into the send_request method
        air_lease_service = kwargs["air_lease_service"]
        request_count = 0
        is_connected = True
        is_connected_count = 0
        # how many times do should the system check the MQTT client's connection status only to find it's not connected?
        connection_down_checks = 3 * TestAirLeaserExternal.FREQUENCY / TestAirLeaserExternal.TIMER_PERIOD
        def on_send_request(request):
            nonlocal request_count
            nonlocal is_connected
            request_count += 1

            # make sure we were given a single air tunnel
            self.assertIsInstance(request, air_lease_requests.Request)
            # make sure the destination matches dest
            nonlocal dest
            actual_dest = BriarLla.from_tup(request.end_position, is_amsl=False)
            self.assertLessEqual(dest.distance(actual_dest.ellipsoid.lla), 0.001)
            self.assertEqual(actual_dest.ellipsoid.lla, dest)
            # make sure the starting position matches pos
            nonlocal pos
            actual_pos = BriarLla.from_tup(request.start_position, is_amsl=False)
            self.assertLessEqual(pos.ellipsoid.lla.distance(actual_pos.ellipsoid.lla), 0.001)

            if request_count == 10:
                # disconnect the MQTT client
                is_connected = False
        
        air_lease_service.send_request.side_effect = on_send_request
        
        def is_connected():
            nonlocal is_connected
            nonlocal is_connected_count

            if is_connected:
                return True
            
            is_connected_count += 1
            if is_connected_count > connection_down_checks:
                is_connected = True
                return True
            return False


        # change our MQTT client so that it appears disconnected
        mqtt = kwargs["mqtt_client"] # this is a mock instance
        mqtt.is_connected.side_effect = is_connected
        self.assertTrue(mqtt.is_connected())

        # build the air leaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )
        # replace the message_queue with a regular queue
        # we will put timer messages in this queue
        air_leaser.message_queue = queue.Queue()

        # How many timer messages should we send? Let's send 100
        total_time = 0
        for _ in range(100):
            msg = self.make_timer_message()
            total_time += msg['data']
            air_leaser.message_queue.put(msg)
        
        total_time_all_messages = total_time
        
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())
        
        outcome = air_leaser.execute(None)
        self.assertEqual(outcome, "error")

        # ok how many send_request calls should we have made?
        # after 10 requests, we'd wait 1 send period before checking mqtt
        total_time -= 1 / TestAirLeaserExternal.FREQUENCY

        # so really it's after 9 "timer triggered requests" that we'd check mqtt
        # and find that it's disconnected. Then we'd let the system check
        # the mqtt connection a total of `missed_timer_messages` more times before we reconnect.
        # we should figure out the worst case scenario. If we only check the mqtt connection
        # once per timer message:
        total_time -= connection_down_checks * TestAirLeaserExternal.TIMER_PERIOD

        # and we don't send our request until we get the next timer message.
        total_time -= TestAirLeaserExternal.TIMER_PERIOD

        
        expected_min_sent = math.floor(total_time * TestAirLeaserExternal.FREQUENCY) + 1


        # now make sure we never called air_lease_service.send_request()
        air_lease_service = kwargs["air_lease_service"]
        # we should not have called send_request() at all
        air_lease_service.send_request.assert_called()
        self.assertLessEqual(expected_min_sent, air_lease_service.send_request.call_count)
        
        # so we establish a minimum number of requests that should have been sent
        # but lets also make sure the code doesn't pass the test by ignoring
        # the mqtt connection status all together.
        # To do this, we will figure out how many requests we would have sent had 
        # we just ignored the mqtt connection status:
        max = math.floor(total_time_all_messages * TestAirLeaserExternal.FREQUENCY) + 1 
        # now make sure we sent fewer than this number of requests:
        self.assertLess(air_lease_service.send_request.call_count, max)


    @patch.object(AirLeaser_module, "rospy")
    @patch.object(AirLeaser_module, "ReadMessagesAirborne")
    @patch.object(AirLeaser_module, "ReadMessages")
    def test_that_we_pause_sending_requests_after_denied(
        self,
        mock_ReadMessages,
        mock_ReadMessagesAirborne,
        mock_rospy
    ):
        """
        This test checks if the system stops sending requests after receiving a denial for an air lease request.

        When an air lease is denied we should not send new requests for a pause
        period. 

        This test creates a scenario where the first request receives an immediate denial.

        We consider the number of timer messages that are needed to trigger the
        AirLeaser to send a request under ordinary circumstances.
        
        We also consider the number of timer messages that are needed to end
        the pause period after a denial.

        We find a number that is in between these two numbers. This number is big enough
        to trigger a request ordinarily, but small enough to _not_ end the pause period.

        We add this special number of timer messsages to verify that we skipped sending at least
        one request. 
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # we need our test scenario to specify the drone's current position
        # and destination.
        # first let's define the current position:
        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        # now for the destination:
        dest = home.ellipsoid.lla.move_ned(10, 10, -5)

        # mock the ReadMessages classes
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        # build the air tunnel function
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest)

        # we want the ability to put a message at the front of the queue
        # so we will use our SpecialQueue class:
        message_queue = SpecialQueue()

        # we need to mock the air_lease_service so that it will send a denial
        def send_denial_response(request):
            # we will put our response in this message queue
            nonlocal message_queue

            # Let's test that AirLeaser gave us the correct type of request
            # we're using a multi-request in this test
            self.assertIsInstance(request, air_lease.MultiRequest)

            # grab the request number because we need to reference it
            # in our response
            req_num = request.requests[0].request_number
            
            # next let's create a response that denies the request
            # since the structure of the response is part of the external
            # interface and we might change the structure of these messages in
            # the future, we will use a method to generate the response:
            msg_data = self.make_response_message(req_num, approved=False)
            assert isinstance(msg_data, dict)

            # now let's create a mock message from an MQTT message
            payload = json.dumps(msg_data)
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC

            air_lease_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=999,
                qos=2,
            )

            # Last we put this response in the front of the message queue
            message_queue.appendleft(air_lease_response)

        # tap into the send_request method
        air_lease_service = kwargs["air_lease_service"]
        air_lease_service.send_request.side_effect = send_denial_response

        # build the air leaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )
        # replace the message_queue with our special one
        air_leaser.message_queue = message_queue

        # How many timer messages should we send?
        # we need to choose this number carefully
        # the number of timer messages that would ordinarily be needed to
        # trigger the AirLeaser to send a request is given by:
        ordinary_timer_msg_count = (
            (1 / TestAirLeaserExternal.FREQUENCY) / TestAirLeaserExternal.TIMER_PERIOD
        )
        # the number of timer messages that would be needed to resume
        # sending requests after a denial is given by:
        wait_period_timer_msg_count = (
            TestAirLeaserExternal.DENIED_PAUSE_TIME / TestAirLeaserExternal.TIMER_PERIOD
        )

        # we want a number that is bigger than `ordinary_timer_msg_count` but smaller than 
        # `wait_period_timer_msg_count`
        # let's take the average
        number_of_timer_messages = (ordinary_timer_msg_count + wait_period_timer_msg_count) / 2
        # but we must have an integer so we will take the floor
        number_of_timer_messages = math.floor(number_of_timer_messages)
        
        # sanity check, make sure we have enough timer messages
        assert number_of_timer_messages > ordinary_timer_msg_count
        # sanity check, make sure we don't have too many timer messages
        assert number_of_timer_messages < wait_period_timer_msg_count
        # this test should be skipped if these conditions are not met
        # these assertions could fail in case we don't have a special wait period

        for _ in range(number_of_timer_messages):
            msg = self.make_timer_message()
            air_leaser.message_queue.put(msg)

        # Now we will add a shutdown message to the end
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())

        # time to run the test
        outcome = air_leaser.execute(None)
        self.assertEqual(air_leaser._current_state.type, air_leaser_interal_state.LEASE_DENIED)
        mock_rospy.logerr.assert_not_called()
        self.assertEqual(outcome, "error")

        # make sure we only called it once:
        air_lease_service.send_request.assert_called_once()

    @patch.object(AirLeaser_module, "rospy")
    @patch.object(AirLeaser_module, "ReadMessagesAirborne")
    @patch.object(AirLeaser_module, "ReadMessages")
    def test_that_how_we_respond_to_deadlock_with_no_deadlock_outcome(
        self,
        mock_ReadMessages,
        mock_ReadMessagesAirborne,
        mock_rospy
    ):
        """
        This test checks if the system ignores the deadlock flag after receiving a denial for an air lease request (with deadlock == true).

        When an air lease is denied we should not send new requests for a pause
        period. 

        This test creates a scenario where the first request receives an immediate denial.

        We consider the number of timer messages that are needed to trigger the
        AirLeaser to send a request under ordinary circumstances.
        
        We also consider the number of timer messages that are needed to end
        the pause period after a denial.

        We find a number that is in between these two numbers. This number is big enough
        to trigger a request ordinarily, but small enough to _not_ end the pause period.

        We add this special number of timer messsages to verify that we skipped sending at least
        one request. 
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # we need our test scenario to specify the drone's current position
        # and destination.
        # first let's define the current position:
        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        # now for the destination:
        dest = home.ellipsoid.lla.move_ned(10, 10, -5)

        # mock the ReadMessages classes
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        # build the air tunnel function
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest)

        # we want the ability to put a message at the front of the queue
        # so we will use our SpecialQueue class:
        message_queue = SpecialQueue()

        # we need to mock the air_lease_service so that it will send a denial
        def send_denial_response(request):
            # we will put our response in this message queue
            nonlocal message_queue

            # Let's test that AirLeaser gave us the correct type of request
            # we're using a multi-request in this test
            self.assertIsInstance(request, air_lease.MultiRequest)

            # grab the request number because we need to reference it
            # in our response
            req_num = request.requests[0].request_number
            
            # next let's create a response that denies the request
            # since the structure of the response is part of the external
            # interface and we might change the structure of these messages in
            # the future, we will use a method to generate the response:
            msg_data = self.make_response_message(req_num, approved=False, deadlock=True)
            assert isinstance(msg_data, dict)

            # now let's create a mock message from an MQTT message
            payload = json.dumps(msg_data)
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC

            air_lease_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=999,
                qos=2,
            )

            # Last we put this response in the front of the message queue
            message_queue.appendleft(air_lease_response)

        # tap into the send_request method
        air_lease_service = kwargs["air_lease_service"]
        air_lease_service.send_request.side_effect = send_denial_response

        # build the air leaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )
        # replace the message_queue with our special one
        air_leaser.message_queue = message_queue

        # How many timer messages should we send?
        # we need to choose this number carefully
        # the number of timer messages that would ordinarily be needed to
        # trigger the AirLeaser to send a request is given by:
        ordinary_timer_msg_count = (
            (1 / TestAirLeaserExternal.FREQUENCY) / TestAirLeaserExternal.TIMER_PERIOD
        )
        # the number of timer messages that would be needed to resume
        # sending requests after a denial is given by:
        wait_period_timer_msg_count = (
            TestAirLeaserExternal.DENIED_PAUSE_TIME / TestAirLeaserExternal.TIMER_PERIOD
        )

        # we want a number that is bigger than `ordinary_timer_msg_count` but smaller than 
        # `wait_period_timer_msg_count`
        # let's take the average
        number_of_timer_messages = (ordinary_timer_msg_count + wait_period_timer_msg_count) / 2
        # but we must have an integer so we will take the floor
        number_of_timer_messages = math.floor(number_of_timer_messages)
        
        # sanity check, make sure we have enough timer messages
        assert number_of_timer_messages > ordinary_timer_msg_count
        # sanity check, make sure we don't have too many timer messages
        assert number_of_timer_messages < wait_period_timer_msg_count
        # this test should be skipped if these conditions are not met
        # these assertions could fail in case we don't have a special wait period

        for _ in range(number_of_timer_messages):
            msg = self.make_timer_message()
            air_leaser.message_queue.put(msg)

        # Now we will add a shutdown message to the end
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())

        # time to run the test
        outcome = air_leaser.execute(None)
        self.assertEqual(air_leaser._current_state.type, air_leaser_interal_state.LEASE_DENIED)
        mock_rospy.logerr.assert_not_called()
        mock_rospy.logwarn.assert_called() # warn about the deadlock
        self.assertEqual(outcome, "error")

        # make sure we only called it once:
        air_lease_service.send_request.assert_called_once()

    @patch.object(AirLeaser_module, "rospy")
    @patch.object(AirLeaser_module, "ReadMessagesAirborne")
    @patch.object(AirLeaser_module, "ReadMessages")
    def test_that_how_we_respond_to_deadlock_when_given_deadlock_outcome(
        self,
        mock_ReadMessages,
        mock_ReadMessagesAirborne,
        mock_rospy
    ):
        """
        This test checks if the system exits with the deadlock outcome if we
        receave a airlease response message that has hte deadlock flag set to true.

        When an air lease is denied, and the deadlock flag is set to true, and we were initalized with the "deadlock" then
        the AirLeaser should exit with the "deadlock" outcome.

        This test creates a scenario where the first request receives an immediate denial.

        We consider the number of timer messages that are needed to trigger the
        AirLeaser to send a request under ordinary circumstances.
        
        We also consider the number of timer messages that are needed to end
        the pause period after a denial.

        We find a number that is in between these two numbers. This number is big enough
        to trigger a request ordinarily, but small enough to _not_ end the pause period.

        We add this special number of timer messsages to verify that we skipped sending at least
        one request. 
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        kwargs["outcomes"] = ["deadlock"]
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # we need our test scenario to specify the drone's current position
        # and destination.
        # first let's define the current position:
        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        # now for the destination:
        dest = home.ellipsoid.lla.move_ned(10, 10, -5)

        # mock the ReadMessages classes
        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        # build the air tunnel function
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest)

        # we want the ability to put a message at the front of the queue
        # so we will use our SpecialQueue class:
        message_queue = SpecialQueue()

        # we need to mock the air_lease_service so that it will send a denial
        def send_denial_response(request):
            # we will put our response in this message queue
            nonlocal message_queue

            # Let's test that AirLeaser gave us the correct type of request
            # we're using a multi-request in this test
            self.assertIsInstance(request, air_lease.MultiRequest)

            # grab the request number because we need to reference it
            # in our response
            req_num = request.requests[0].request_number
            
            # next let's create a response that denies the request
            # since the structure of the response is part of the external
            # interface and we might change the structure of these messages in
            # the future, we will use a method to generate the response:
            msg_data = self.make_response_message(req_num, approved=False, deadlock=True)
            assert isinstance(msg_data, dict)

            # now let's create a mock message from an MQTT message
            payload = json.dumps(msg_data)
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC

            air_lease_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=999,
                qos=2,
            )

            # Last we put this response in the front of the message queue
            message_queue.appendleft(air_lease_response)

        # tap into the send_request method
        air_lease_service = kwargs["air_lease_service"]
        air_lease_service.send_request.side_effect = send_denial_response

        # build the air leaser object
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )
        # replace the message_queue with our special one
        air_leaser.message_queue = message_queue

        # How many timer messages should we send?
        # we need to choose this number carefully
        # the number of timer messages that would ordinarily be needed to
        # trigger the AirLeaser to send a request is given by:
        ordinary_timer_msg_count = (
            (1 / TestAirLeaserExternal.FREQUENCY) / TestAirLeaserExternal.TIMER_PERIOD
        )
        # the number of timer messages that would be needed to resume
        # sending requests after a denial is given by:
        wait_period_timer_msg_count = (
            TestAirLeaserExternal.DENIED_PAUSE_TIME / TestAirLeaserExternal.TIMER_PERIOD
        )

        # we want a number that is bigger than `ordinary_timer_msg_count` but smaller than 
        # `wait_period_timer_msg_count`
        # let's take the average
        number_of_timer_messages = (ordinary_timer_msg_count + wait_period_timer_msg_count) / 2
        # but we must have an integer so we will take the floor
        number_of_timer_messages = math.floor(number_of_timer_messages)
        
        # sanity check, make sure we have enough timer messages
        assert number_of_timer_messages > ordinary_timer_msg_count
        # sanity check, make sure we don't have too many timer messages
        assert number_of_timer_messages < wait_period_timer_msg_count
        # this test should be skipped if these conditions are not met
        # these assertions could fail in case we don't have a special wait period

        for _ in range(number_of_timer_messages):
            msg = self.make_timer_message()
            air_leaser.message_queue.put(msg)

        # Now we will add a shutdown message to the end
        air_leaser.message_queue.put(mock_types.mock_shutdown_message())

        # time to run the test
        outcome = air_leaser.execute(None)
        mock_rospy.logwarn.assert_called() # we should have logged a warning for the deadlock
        mock_rospy.logerr.assert_not_called()
        self.assertEqual(outcome, "deadlock")

        # make sure we only called it once:
        air_lease_service.send_request.assert_called_once()

    @patch.object(AirLeaser_module.random, 'uniform')
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_resume_sending_requests_after_a_denial(self, mock_ReadMessages, mock_ReadMessagesAirborne, mock_uniform):
        """Test that we resume sending air lease requests after getting denied.

        To pass this test, the system must send a follow up air lease request
        after getting denied. We will mock the air_lease_service so that it
        sends a denial response (only after the inital request).

        We will make sure there are plenty of timer messages.
        We need enough of them to end the pause period and trigger a request.
        """
        mock_uniform.return_value = 1.0
        kwargs = mock_types.mock_mission_builder().build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)

        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        dest = home.ellipsoid.lla.move_ned(10, 100, -5)
        dest = BriarLla.from_lla(dest, is_amsl=False)

        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        message_queue = SpecialQueue()
        is_denied_once = False

        def send_denial_once(request):
            nonlocal message_queue
            nonlocal is_denied_once
            if is_denied_once:
                return
            is_denied_once = True

            self.assertIsInstance(request, air_lease.MultiRequest)
            req_num = request.requests[0].request_number
            msg_data = self.make_response_message(req_num, approved=False)
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC
            payload = json.dumps(msg_data)
            air_lease_response = mock_types.mock_mqtt_message(
                msg_sender,
                topic=mqtt_topic,
                payload=payload,
                mid=111,
                qos=2,
            )
            message_queue.appendleft(air_lease_response)
        
        air_lease_service = kwargs["air_lease_service"]
        air_lease_service.send_request.side_effect = send_denial_once

        air_tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest.ellipsoid.lla)

        air_leaser = AirLeaser(
            tunnel_func=air_tunnel_func,
            is_flying=True,
            **kwargs
        )

        air_leaser.message_queue = message_queue
        
        # next we need carefully calculate the number of timer messages we want to use.
        # we need at a minimum, enough to trigger a request after the wait period.
        wait_period_count = TestAirLeaserExternal.DENIED_PAUSE_TIME / TestAirLeaserExternal.TIMER_PERIOD
        wait_period_count = math.ceil(wait_period_count)
        # now we should decide on the number of additional messages we'd like to send
        # once we resume. Let's go with 10
        ordinary_period_count = TestAirLeaserExternal.FREQUENCY / TestAirLeaserExternal.TIMER_PERIOD
        additional_messages = 10
        # number of timer messages to trigger 10 messages
        sending_period = math.ceil(additional_messages*ordinary_period_count)
        # now we can calculate the total number of messages we need
        n_messages = wait_period_count + sending_period

        for _ in range(n_messages):
            msg = self.make_timer_message()
            message_queue.put(msg)

        message_queue.put(mock_types.mock_shutdown_message())

        outcome = air_leaser.execute(None)
        self.assertEqual(outcome, "error")

        # so the total number of expected messages is 10 + 1 for initial request
        air_leaser.air_lease_service.send_request.assert_called()
        self.assertEqual(air_leaser.air_lease_service.send_request.call_count, additional_messages + 1)

    @patch.object(AirLeaser_module.random, 'uniform')
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_that_we_ignore_irrelevant_air_lease_responses(self, mock_ReadMessages, mock_ReadMessagesAirborne, mock_uniform):
        """Test that we do not respond to irrelevent air lease responses.

        An irrelevent response is one that meets any of the following conditions:
        - it has the wrong drone name
        - it has the wrong request number

        So if the drone name doesn't match ours or the request number doesn't
        match our request then we should ignore the message.

        To pass this test, the system must send the expected number air lease requests
        And the overall outcome must be "error"

        We will put messages that approve the wrong drone but the right request number.
        We will put messages that approve the wrong request number but the right drone.
        we will put messages that deny the wrong drone.
        we will put messages that deny the wrong request number.

        Since all of these messages address irrelevant requests, we should ignore them.
        Therefore we will expect a number of requests oroportional to the number of timer messages
        """
        mock_uniform.return_value = 1.0
        kwargs = mock_types.mock_mission_builder().build_kwargs()
        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)

        pos = home.ellipsoid.lla.move_ned(10, 0, -10)
        pos = BriarLla.from_lla(pos, is_amsl=False)
        dest = home.ellipsoid.lla.move_ned(10, 100, -5)
        dest = BriarLla.from_lla(dest, is_amsl=False)

        mock_ReadMessages.return_value = mock_types.mock_ReadMessages(pos)
        mock_ReadMessagesAirborne.return_value = mock_types.mock_ReadMessages(pos)

        message_queue = SpecialQueue()
        message_queue.put(mock_types.mock_shutdown_message())

        is_done = False

        def send_irrelevant_responses(request):
            """Here we will put nearlly all messages in the queue

            We will create a bunch of timer messages.
            We will intersperse the irrelevant responses in them.
            specifically we want some timer messages at the start,
            some timer messsages at the end.
            And in between we will put the irrelevant responses at regular
            intervals.
            """
            nonlocal message_queue
            nonlocal is_done
            if is_done:
                return
            
            self.assertIsInstance(request, air_lease_requests.MultiRequest)
            req_num = request.requests[0].request_number
            drone_id = request.requests[0].drone_id
            wrong_drone_id = f"NOT_{drone_id}"
            wrong_req_num = req_num + 2000
            # we must create 4 irrelevant responses:
            air_lease_response_data = [
                # 1. approve the wrong drone but the right request number.
                self.make_response_message(req_num, approved=True, drone_id=wrong_drone_id),

                # 2. approve the wrong request number but the right drone.
                self.make_response_message(wrong_req_num, approved=True, drone_id=drone_id),

                # 3. deny the wrong drone but the right request number
                self.make_response_message(req_num, approved=False, drone_id=wrong_drone_id),

                # 4. deny the wrong request number but the right drone.
                self.make_response_message(wrong_req_num, approved=False, drone_id=drone_id),

            ]

                
            msg_sender = TestAirLeaserExternal.AIR_LEASE_MESSAGE_SENDER
            mqtt_topic = TestAirLeaserExternal.AIR_LEASE_RESPONSE_MQTT_TOPIC
            msg_id = 111
            air_lease_messages = []
            for msg_data in air_lease_response_data:
                payload = json.dumps(msg_data)
                air_lease_response = mock_types.mock_mqtt_message(
                    msg_sender,
                    topic=mqtt_topic,
                    payload=payload,
                    mid=msg_id,
                    qos=2,
                )
                msg_id += 1
                air_lease_messages.append(air_lease_response)

            # now that we have the air lease messages, we can create our entire message queue
            # we will create a list
            # we want to trigger 2 messages before the system receives the first bad response
            # after we append each bad response we want to trigger 3 messages
            # so we have enough timer messages to send (2 + 3*4) messages 
            # that's 14 messages

            ordinary_period_count = TestAirLeaserExternal.FREQUENCY / TestAirLeaserExternal.TIMER_PERIOD

            before = math.ceil(ordinary_period_count * 2)
            between = math.ceil(ordinary_period_count * 3)

            all_messages = []

            timer_msg_count = 0

            for _ in range(before):
                msg = self.make_timer_message()
                timer_msg_count += 1
                all_messages.append(msg)
            
            for msg in air_lease_messages:
                all_messages.append(msg)
                for _ in range(between):
                    msg = self.make_timer_message()
                    timer_msg_count += 1
                    all_messages.append(msg)
            
            assert round(timer_msg_count / 4) == (3*4 + 2)
            
            # now we want all our messages to be inserted before the shutdown message
            # so we will pop the shutdown message off the queue
            # then we will add all our messages
            # then we will add the shutdown message back
            
            shutdown_msg = message_queue.get()
            for msg in all_messages:
                message_queue.put(msg)
            message_queue.put(shutdown_msg)

            is_done = True
        
        air_lease_service = kwargs["air_lease_service"]
        air_lease_service.send_request.side_effect = send_irrelevant_responses

        air_tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest.ellipsoid.lla)

        air_leaser = AirLeaser(
            tunnel_func=air_tunnel_func,
            is_flying=True,
            **kwargs
        )

        air_leaser.message_queue = message_queue

        outcome = air_leaser.execute(None)
        self.assertEqual(outcome, "error")

        # so the total number of expected messages is
        # 1 initial request
        # 2 before the first irrelevant response
        # 3 after each of the 4 irrelevant responses
        # so 1 + 2 + 3*4 = 15
        expected_number_sent = 1 + 2 + 3*4
        assert expected_number_sent == 15
        air_leaser.air_lease_service.send_request.assert_called()
        self.assertEqual(air_leaser.air_lease_service.send_request.call_count, expected_number_sent)
    
    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_communicate_route_complete(self, mock_ReadMessages, mock_ReadMessagesAirborne):
        kwargs = mock_types.mock_mission_builder().build_kwargs()
        air_lease_service = kwargs["air_lease_service"]

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )

        dest = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest.ellipsoid.lla)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        air_leaser.communicate_route_complete(current_position=mock_current_position)

        air_lease_service.send_done.assert_called_once_with(
            position=[
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            ]
        )

    @patch.object(AirLeaser_module, 'ReadMessagesAirborne')
    @patch.object(AirLeaser_module, 'ReadMessages')
    def test_communicate_route_complete_no_position(self, mock_ReadMessages, mock_ReadMessagesAirborne):
        kwargs = mock_types.mock_mission_builder().build_kwargs()
        air_lease_service = kwargs["air_lease_service"]
        dest = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        tunnel_func = air_lease.make_waypoint_multi_air_tunnel_func(dest.ellipsoid.lla)
        
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        air_leaser.communicate_route_complete()

        air_lease_service.send_done.assert_called_once_with()
    
    def test_land_method(self):
        """
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()

        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        pos = home.ellipsoid.lla.move_ned(0, 0, -15)
        land_tunnel_pos = pos.move_ned(0, 0, pos.alt) # move to the point where our Ellipsoidal altitude is zero

        tunnel_func = air_lease.make_waypoint_air_tunnel_func(land_tunnel_pos)
        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            is_flying=True,
            **kwargs
        )

        outcome = air_leaser.land()
        self.assertEqual(outcome, "succeeded")

        air_leaser_service = kwargs["air_lease_service"]
        air_leaser_service.land.assert_called_once_with()
