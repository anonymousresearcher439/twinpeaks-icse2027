import json
from typing import List
from dataclasses import asdict
import unittest
from unittest.mock import Mock, patch

from droneresponse_mathtools import Lla
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition


from dr_onboard_autonomy.airlease import protocol as airleaser_protocol
from dr_onboard_autonomy.airlease.protocol import AirLeaseOutcome
from dr_onboard_autonomy.airlease.messages import (
    Cleanup,
    HoverRequest,
    Land,
    MultiRequest as MultiTunnelRequest,
    Request as SingleTunnelRequest,
)
from dr_onboard_autonomy.airlease.airspace_volume import AirspaceVolume, AirspaceSegment, AirspaceVolumeName

from . import mock_types


class AirLeaseGroundServiceHelper:
    """Helper class to simulate ground service responses for testing.
    
    This helper mimics what the real air lease ground service would do
    for a single drone scenario. It approves requests if:
    1. request_number is valid (greater than current_lease)
    2. current_lease value is correct
    
    Note: This simplified version does not check for overlapping airspace.
    """
    
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.current_lease_id = 0  # Start with no active lease
        self.largest_request_number_seen = 0
    
    def process_request(self, request: dict) -> dict:
        """Process an air lease request and return a response.
        
        Args:
            request: Dictionary containing either a single Request or MultiRequest
        
        Returns:
            Dictionary with response format:
            {
                "drone_id": str,
                "request_number": int,
                "current_lease": int,
                "approved": bool,
                "deadlock": bool,
            }
        """
        # Check if this is a MultiRequest (has "requests" key) or single Request
        if "requests" in request:
            return self._process_multi_request(request)
        else:
            return self._process_single_request(request)
    
    def _process_single_request(self, request: dict) -> dict:
        """Process a single tunnel request."""
        drone_id = request["drone_id"]
        request_number = request["request_number"]
        current_lease = request["current_lease"]
        
        # Validate drone_id matches
        if drone_id != self.drone_id:
            return self._create_response(request_number, False, False)
        
        # Check if request_number is valid (must be greater than current_lease)
        if request_number <= current_lease:
            return self._create_response(request_number, False, False)
        
        # Check if current_lease matches our records
        if current_lease != self.current_lease_id:
            return self._create_response(request_number, False, False)
        
        # Check if request_number is greater than any we've seen before
        if request_number <= self.largest_request_number_seen:
            return self._create_response(request_number, False, False)
        
        # Request is approved - update our state
        self.largest_request_number_seen = request_number
        self.current_lease_id = request_number
        
        return self._create_response(request_number, True, False)
    
    def _process_multi_request(self, request: dict) -> dict:
        """Process a multi-tunnel request."""
        requests = request["requests"]
        
        if not requests:
            return self._create_response(0, False, False)
        
        # Use the first request for validation and response
        first_request = requests[0]
        drone_id = first_request["drone_id"]
        request_number = first_request["request_number"]
        current_lease = first_request["current_lease"]
        
        # Validate all requests have the same drone_id and current_lease
        for req in requests:
            if req["drone_id"] != drone_id or req["current_lease"] != current_lease:
                return self._create_response(request_number, False, False)
        
        # Validate drone_id matches
        if drone_id != self.drone_id:
            return self._create_response(request_number, False, False)
        
        # Check if request_number is valid (must be greater than current_lease)
        if request_number <= current_lease:
            return self._create_response(request_number, False, False)
        
        # Check if current_lease matches our records
        if current_lease != self.current_lease_id:
            return self._create_response(request_number, False, False)
        
        # Check if request_number is greater than any we've seen before
        if request_number <= self.largest_request_number_seen:
            return self._create_response(request_number, False, False)
        
        # Request is approved - update our state
        self.largest_request_number_seen = request_number
        self.current_lease_id = request_number
        
        return self._create_response(request_number, True, False)
    
    def _create_response(self, request_number: int, approved: bool, deadlock: bool) -> dict:
        """Create a response dictionary."""
        return {
            "drone_id": self.drone_id,
            "request_number": request_number,
            "current_lease": self.current_lease_id,
            "approved": approved,
            "deadlock": deadlock,
        }

def mock_gcs_mqtt():
    return mock_types.mock_mqtt_client(
        uav_name="TEST DRONE",
        broker_address="mqtt",
        local_ip="1.2.3.4"
    )

default_start_pos = LlaPosition(latitude=1, longitude=1, altitude=1, datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84)
default_end_pos = LlaPosition(latitude=1.00001, longitude=1, altitude=1, datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84)

def create_segment(
        start_pos: LlaPosition=default_start_pos,
        end_pos: LlaPosition=default_end_pos,
        radius: float = 5
    ) -> AirspaceSegment:
    return AirspaceSegment(
        start=start_pos,
        end=end_pos,
        radius=radius
    )


def create_airspace_volume(num_segments: int = 3) -> AirspaceVolume:
    segments = [create_segment() for _ in range(num_segments)]
    return segments

def create_response(request_number: int, current_lease: int, approved: bool, deadlock: bool) -> SingleTunnelRequest:
    uav_name = "TEST DRONE"
    response = {
        "drone_id": uav_name,
        "request_number": request_number,
        "current_lease": current_lease,
        "approved": approved,
        "deadlock": deadlock,
    }
    msg_sender_dict = mock_types.mock_mqtt_message(
        msg_sender_name="airlease_status",
        topic=f"drone/{uav_name}/airlease/status",
        payload=json.dumps(response)
    )
    mqtt_message = msg_sender_dict['data']
    return mqtt_message
    

def my_callback(name: AirspaceVolumeName, airspace: AirspaceVolume, outcome: AirLeaseOutcome):
    pass

class TestAirLeaserProtocol(unittest.TestCase):
    def test_initial_state(self):
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        self.assertEqual(model.pending_requests, {})
        self.assertEqual(model.uav_id, "TEST DRONE")
        self.assertEqual(model.mqtt, mqtt_gcs)
        self.assertEqual(model.lease_id, 0)
        self.assertIsNone(model.target_airspace)
        self.assertIsNone(model._current_airspace)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_airleaser_protocol_sends_single_tunnel_and_enters_waiting_response(self, mock_next_request_number):
        mqtt_gcs = mock_gcs_mqtt()
        time_ns = 1741057961133953128
        mock_next_request_number.return_value = time_ns // 1_000_000
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.state, airleaser_protocol.States.IDLE)

        r1 = create_airspace_volume(1)
        model.update_airspace(r1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        self.assertEqual(model.target_airspace.airspace, r1)

        # Need to make sure we have a pending request
        time_millis = int(time_ns // 1_000_000)
        expected_request_number = time_millis
        self.assertIn(expected_request_number, model.pending_requests)
        self.assertEqual(model.pending_requests[expected_request_number].airspace, r1)
        # make sure we called mqtt.publish
        mqtt_gcs.publish.assert_called_once()
        # We need to make sure we sent a SingleTunnelRequest

        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['topic'], "airlease/request")
        self.assertEqual(call_args.kwargs['qos'], 0)

        expected_request = SingleTunnelRequest(
            drone_id="TEST DRONE",
            start_position=(1, 1, 1),
            current_lease=0,
            end_position=(1.00001, 1, 1),
            radius=5,
            request_number=time_millis
        )

        expected_dict = asdict(expected_request)

        self.assertIsInstance(call_args.kwargs['data'], dict)
        self.assertEqual(call_args.kwargs['data'], expected_dict)

        # double check that these fields are the same
        self.assertEqual(model.uav_id, "TEST DRONE")
        self.assertEqual(model.mqtt, mqtt_gcs)
        self.assertEqual(model.lease_id, 0)
        self.assertIsNone(model._current_airspace)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_airleaser_protocol_sends_multi_tunnel_and_enters_waiting_response(self, mock_next_request_number):
        mqtt_gcs = mock_gcs_mqtt()
        time_ns = 1741057961133953128
        first_req_number = time_ns // 1_000_000
        mock_next_request_number.return_value = first_req_number
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.state, airleaser_protocol.States.IDLE)

        r1 = create_airspace_volume(2)
        name = model.update_airspace(r1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        self.assertEqual(model.target_airspace.airspace, r1)
        self.assertEqual(model.target_airspace.name, name)
        # get the request number from the MQTT data
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], first_req_number)

        # Need to make sure we have a pending request
        expected_request_number = first_req_number
        self.assertIn(expected_request_number, model.pending_requests)
        self.assertEqual(model.pending_requests[expected_request_number].airspace, r1)
        # make sure we called mqtt.publish
        mqtt_gcs.publish.assert_called_once()
        # We need to make sure we sent a SingleTunnelRequest

        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['topic'], "airlease/request")
        self.assertEqual(call_args.kwargs['qos'], 0)

        expected_requests = [
                SingleTunnelRequest(
                drone_id="TEST DRONE",
                current_lease=0,
                start_position=(1, 1, 1),
                end_position=(1.00001, 1, 1),
                radius=5,
                request_number=first_req_number
            ),
            SingleTunnelRequest(
                drone_id="TEST DRONE",
                current_lease=0,
                start_position=(1, 1, 1),
                end_position=(1.00001, 1, 1),
                radius=5,
                request_number=first_req_number + 1
            )
        ]

        expected_request = MultiTunnelRequest(requests=expected_requests)

        expected_dict = asdict(expected_request)

        self.assertIsInstance(call_args.kwargs['data'], dict)
        self.assertEqual(call_args.kwargs['data'], expected_dict)
        

    def test_airleaser_protocol_updates_current_lease(self):
        """Test what happens when we get a response that approves the lease
        """
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        
        mock_callback = Mock(wraps=my_callback)

        r1 = create_airspace_volume(1)
        expected_name = model.update_airspace(airspace=r1, callback=mock_callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        # Need to make sure we have a pending request
        # check by looking that we have 1 key in pending requests
        self.assertEqual(len(model.pending_requests), 1)
        
        request_number, airspace_record = list(model.pending_requests.items())[0]
        self.assertEqual(airspace_record.name, expected_name)
        self.assertEqual(airspace_record.airspace, r1)

        # make sure we called mqtt.publish
        mqtt_gcs.publish.assert_called_once()

        # ok let's create a mock response
        mock_response_payload = {
            'drone_id': "TEST DRONE",
            'request_number': request_number,
            'current_lease': request_number,
            'approved': True,
            'deadlock': False
        }

        mock_response = mock_types.mock_mqtt_message(
            msg_sender_name="airlease_status",
            topic=f"drone/TEST DRONE/airlease/status",
            payload=json.dumps(mock_response_payload)
        )['data']

        model.receive_response(mock_response)
        model.receive_response(mock_response)

        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        expected_outcome =  airleaser_protocol.AirLeaseOutcome(True, False)
        mock_callback.assert_called_once_with(expected_name, r1, expected_outcome)

        self.assertEqual(model.lease_id, request_number)
    
    def test_airleaser_protocol_sends_and_enters_disconnected_when_mqtt_is_down(self):
        # let's simulate what happens when the mqtt connection is down
        mqtt_gcs = mock_gcs_mqtt()
        mqtt_gcs.is_connected.return_value = False
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        r1 = create_airspace_volume(4)
        model.update_airspace(airspace=r1)
        self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        self.assertIsInstance(model.target_airspace, airleaser_protocol.AirspaceRecord)
        self.assertIsInstance(model.target_airspace.name, AirspaceVolumeName)

        self.assertEqual(model.target_airspace.airspace, r1)

        # make sure we don't have a pending request
        #make sure pending requests is empty!!!!
        self.assertEqual(model.pending_requests, {})
        # make sure we never called mqtt.publish
        mqtt_gcs.publish.assert_not_called()
    
    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_airleaser_protocol_updates_lease_with_many_pending_requests(self, mock_next_request_number):
        # The number of times we're going to change our target airspace
        num_airspace_volumes = 5

        all_req_numers = [
            1741100313659 + 20_000 * i for i in range(num_airspace_volumes*2) # we probably don't need 2x but I just want to make sure we have enough
        ]
        mock_next_request_number.side_effect = all_req_numers
        
        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.lease_id, 0) # this value is a big part of the test that it starts at 0
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)

        airspace_volumes = [create_airspace_volume(3) for _ in range(num_airspace_volumes)]
        for i, next_request in enumerate(airspace_volumes):
            model.update_airspace(next_request)
            self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
            self.assertEqual(model.target_airspace.name, i + 1)
            self.assertEqual(model.target_airspace.airspace, next_request)
            # let's check the pending requests next
            # the request number should be the last value in time_ns_return_vals
            expected_request_number = all_req_numers[i]
            self.assertIn(expected_request_number, model.pending_requests)
            self.assertEqual(model.pending_requests[expected_request_number].airspace, next_request)
            # make sure we called mqtt.publish
            mqtt_gcs.publish.assert_called_once()
            mqtt_gcs.reset_mock()
        
        # Now we take a request from the middle of the list and simulate a response
        expected_airspace_index = 2
        expected_airspace = airspace_volumes[expected_airspace_index]
        expected_lease_id = all_req_numers[expected_airspace_index]
        
        # now we craft an Airlease Response
        mqtt_msg = create_response(expected_lease_id, expected_lease_id, True, False)

        model.receive_response(mqtt_msg)

        # we should still be wait because the target airspace is not the one that was approved
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        self.assertEqual(model.lease_id, expected_lease_id)
        self.assertEqual(model.airspace, expected_airspace)
        self.assertEqual(model.target_airspace.airspace, airspace_volumes[-1])
        self.assertEqual(model.target_airspace.name, num_airspace_volumes)
        # make sure we cleaned up the pending requests
        for req_num in all_req_numers[:expected_airspace_index+1]:
            self.assertNotIn(req_num, model.pending_requests)
        for request_number in model.pending_requests.keys():
            self.assertGreater(request_number, expected_lease_id)
        
        still_pending_index = expected_airspace_index+1
        for expected_req_num, airspace in zip(all_req_numers[still_pending_index:], airspace_volumes[still_pending_index:]):
            self.assertIn(expected_req_num, model.pending_requests)
            self.assertEqual(model.pending_requests[expected_req_num].airspace, airspace)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_update_current_lease_from_int(self, mock_next_request_number):
        """In this test we're going to simulate what would happen if a previous instance of dr_onboard
        had flown and Acquired air leases. We're going to start off thinking our current lease ID is 0
        but then we're going to get a response from the GCS that our lease is something else.

        This could happen in simulation if we're flying the drone and then we decide to kill dr_onboard
        but we don't kill the ground station.

        In that case the ground station will still think we have a lease and it will deny
        our request for a new lease because we're not referencing the current lease ID.
        """

        mock_req_numbers = [
            1741100313659 + 20_000 * i for i in range(10)
        ]
        mock_next_request_number.side_effect = mock_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertEqual(model.lease_id, 0)
        airspace_volumes = [create_airspace_volume(i) for i in range(2, 13, 2)] # 2, 4, 6,... 12
        mock_callbacks = [Mock(wraps=my_callback) for _ in range(len(airspace_volumes))]

        for next_airspace, mock_callback, expected_req_number in zip(airspace_volumes, mock_callbacks, mock_req_numbers):
            model.update_airspace(next_airspace, mock_callback)
            self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
            self.assertEqual(model.target_airspace.airspace, next_airspace)
            self.assertIn(expected_req_number, model.pending_requests)
            mqtt_gcs.publish.assert_called_once()
            # we need to look at the request and make sure current lease is set to 0
            call_args = mqtt_gcs.publish.call_args
            self.assertEqual(call_args.kwargs['data']['requests'][0]['current_lease'], 0)
            self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], expected_req_number)
            mqtt_gcs.reset_mock()
        
        # Ok now we take a response from the middle and deny it.
        # we expect a few things:
        # current lease ID should update to whatever the response says it is
        # all pending requests with a number less than request_number we're responding to should be removed
        
        # Now we take a request from the middle of the list make sure we update the current lease correctly
        response_index = len(airspace_volumes) // 2
        response_request_number = mock_req_numbers[response_index]
        expected_current_lease_id = 1741080313659 # 1741100313659 - 20_000_000 # 20 seconds before the first request number
        mqtt_msg = create_response(response_request_number, expected_current_lease_id, False, False)

        model.receive_response(mqtt_msg)

        self.assertEqual(model.lease_id, expected_current_lease_id)
        # Make sure we're in waiting we do not go to denied because the response
        # isn't talking about the target airspace, it's talking a previous
        # target airspace
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE) 
        # since the response names a lease ID we've never seen before
        # all we can do is update the current lease ID
        # therefore the current airspace should be None
        self.assertIsNone(model.airspace)

        # make sure we cleaned up the pending requests with a request_number less than response_request_number
        for remaining_req_num in model.pending_requests.keys():
            self.assertGreater(remaining_req_num, response_request_number)
        
        # make sure we didn't call any of the callbacks
        # Note this design decision is up for debate
        # these requests were technically denied but we didn't call the callbacks
        # because they were denied for a reason that had nothing to do with the airspace
        # therefore I'm going to not call the callbacks
        # for now the callback is only called if the airspace is approved
        # or if it is explicitly denied and it's the target airspace and the current lease is correct
        for mock_callback in mock_callbacks:
            mock_callback.assert_not_called()
    
    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_that_we_called_the_callback_when_the_airspace_is_denied(self, mock_next_request_number):
        """so the callback behavior for a denied request is "best effort"

        we will only call if:
            the response is about the target airspace
            the current lease was correct

        and the current lease is correct
        If we are denied because the airspace is not available,
        then we should call the callback

        Really seems like there's gonna be some cases where we're denied
        but we don't call the callback because we didn't get a respose

        This problem is only for denied requests because approved requests
        will always call the callback
        """
        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        all_request_numbers = [
            (1741109875478505419 // 1_000_000) + i* 1000 for i in range(10)
        ]
        mock_next_request_number.side_effect = all_request_numbers

        airspace = create_airspace_volume(1)
        callback = Mock(name="callback")
        model.update_airspace(airspace, callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        # let's get the request number from the mqtt message
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        actual_request_num = call_args.kwargs['data']['request_number']
        
        self.assertEqual(actual_request_num, all_request_numbers[0])

        # Now we simulate a response that denies the request
        mqtt_msg = create_response(actual_request_num, 0, False, False)
        model.receive_response(mqtt_msg)
        self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        self.assertEqual(model.lease_id, 0)
        self.assertIsNone(model.airspace)
        callback.assert_called_once_with(1, airspace, airleaser_protocol.AirLeaseOutcome(False, False))

    def test_that_we_do_not_call_the_callback_when_the_airspace_is_denied_for_sync_issue(self):
        """We simulate a deny response because the current lease is wrong
        """
        current_lease = 1741110283560 # this is the current lease but the model doesn't know it yet
        # Note if we don't mock time then the request number will be the same
        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.lease_id, 0) # we will check on this again later

        airspace = create_airspace_volume(1)
        callback = Mock(name="callback")
        model.update_airspace(airspace, callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        # let's get the request number from the mqtt message
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        request_num1 = call_args.kwargs['data']['request_number']
        mqtt_gcs.reset_mock()

        # Now we simulate a response that denies the request
        mqtt_msg = create_response(request_num1, current_lease, False, False)
        model.receive_response(mqtt_msg)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        self.assertEqual(model.lease_id, current_lease)
        self.assertIsNone(model.airspace)
        callback.asssert_not_called()
        # make sure publish was called AGAIN!
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['current_lease'], current_lease)

        request_number2 = call_args.kwargs['data']['request_number']

        # make sure we cleaned up the pending requests
        self.assertNotIn(request_num1, model.pending_requests)
        # make sure we have a new pending request
        self.assertEqual(len(model.pending_requests), 1)
        self.assertIn(request_number2, model.pending_requests)

    def test_that_we_do_not_call_the_callback_when_the_airspace_is_denied_for_sync_issue2(self):
        """We simulate a deny response because the current lease is wrong
        But we will change our target airspace 3 times before we get the response
        And the response will be for the second request
        """
        current_lease = 1741110283560 # this is the current lease but the model doesn't know it yet
        # Note if we don't mock time then the request number will be the same
        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.lease_id, 0) # we will check on this again later

        all_callbacks = []
        req_numbers = []
        for i in range(3):
            airspace = create_airspace_volume(1)
            callback = Mock(name="callback")
            all_callbacks.append(callback)
            model.update_airspace(airspace, callback)
            self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
            # let's get the request number from the mqtt message
            mqtt_gcs.publish.assert_called_once()
            call_args = mqtt_gcs.publish.call_args
            req_numbers.append(call_args.kwargs['data']['request_number'])
            # make sure the current_lease is set to zero
            self.assertEqual(call_args.kwargs['data']['current_lease'], 0)
            mqtt_gcs.reset_mock()

        # Now we simulate a response that denies the 2nd request
        req_num2 = req_numbers[1]
        mqtt_msg = create_response(req_num2, current_lease, False, False)
        model.receive_response(mqtt_msg)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        self.assertEqual(model.lease_id, current_lease)
        self.assertIsNone(model.airspace)
        for cb in all_callbacks:
            cb.asssert_not_called()
        # make sure publish was called AGAIN! If we see a sync issue we should
        # publish immediately
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['current_lease'], current_lease)

        req_numbers.append(call_args.kwargs['data']['request_number'])
        
        # I want to partition the request numbers
        # I want two lists one called before and one called after
        # the split happens at index 1. So index 0 and 1 go in before
        # and the rest go in after
        before, after = req_numbers[:2], req_numbers[2:]

        # make sure we cleaned up the pending requests
        for request_num in before:
            self.assertNotIn(request_num, model.pending_requests)
        # make sure the other requests are still pending
        for request_num in after:
            self.assertIn(request_num, model.pending_requests)


    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_cleanup_pending(self, mock_next_request_number):

        all_req_numbers = [
            1741100313659 + 20_000 * i for i in range(10)
        ]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, _ = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        self.assertEqual(model.lease_id, 0)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        airspace_volumes = [create_airspace_volume(i) for i in range(1, 6)]
        for next_request in airspace_volumes:
            model.update_airspace(next_request)
            self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
            self.assertEqual(model.target_airspace.airspace, next_request)
        
        # Now we take a request from the middle of the list to make sure we
        # update the current lease correctly
        lease_index = 2 # for this test we arbitrarily choose 2 as the lease we want to approve
        lease_id = all_req_numbers[lease_index]
        model._update_current_lease(lease_id)
        model._cleanup_pending_requests(lease_id)

        # so we need to make sure all the pending requests with a number less than the lease_id are cleaned up
        for airspace, req_num in zip(airspace_volumes, all_req_numbers):
            if req_num <= lease_id:
                self.assertNotIn(req_num, model.pending_requests)
            else:
                self.assertIn(req_num, model.pending_requests)
        self.assertEqual(model.airspace, airspace_volumes[lease_index])

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_receive_response_1(self, mock_next_request_number):
        """
        Test the case where the response is approved

        We will simulate the following:
        update our target airlease
        receive a response from the GCS
        """
        req_number = 1000
        mock_next_request_number.return_value = req_number
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace1 = create_airspace_volume(1)
        


        mock_callback = Mock(wraps=my_callback)

        airspace_name = model.update_airspace(airspace=airspace1, callback=mock_callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()

        msg = create_response(req_number, req_number, True, False)

        model.receive_response(msg)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        mock_callback.assert_called_once_with(airspace_name, airspace1, airleaser_protocol.AirLeaseOutcome(True, False))
        self.assertEqual(model.target_airspace.name, airspace_name)
        self.assertNotIn(req_number, model.pending_requests)
        # make sure pending requests is empty
        self.assertEqual(model.pending_requests, {})

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_receive_response_2(self, mock_next_request_number):
        """
        Test the case where the lease is denied

        We will simulate the following:
        update our target airspace and send one request
        receive a response from the GCS that denies it
        """
        req_number = 1000
        mock_next_request_number.return_value = req_number
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        r1 = create_airspace_volume(24)
        mock_callback = Mock(wraps=my_callback)
        name = model.update_airspace(airspace=r1, callback=mock_callback)
        
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()

        # Now we simulate the response from the GCS
        msg = create_response(req_number, 0, False, False)
        model.receive_response(msg)

        self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        mock_callback.assert_called_once()
        self.assertEqual(model.target_airspace.name, name)
        self.assertEqual(model.target_airspace.airspace, r1)
        self.assertNotIn(req_number, model.pending_requests)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_receive_response_3(self, next_request_number):
        """
        Test the case where:
        1. we ask for airspace 1
        2. we ask for airspace 2
        3. we receive a response for airspace 1 (it's approved)

        expected behavior:
            - we should be in WAITING_RESPONSE
            - we should have pending request for airspace 2
            - we should have a current airspace from airspace 1
            - we should have called the callback for airspace 1
            - we should have not called the callback for airspace 2
        """
        all_req_numbers = [1000, 2000, 3000]
        req_num1, req_num2, req_num3 = all_req_numbers
        next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertEqual(model.lease_id, 0)
        self.assertIsNone(model.airspace)

        airspace1 = create_airspace_volume(1) 
        airspace2 = create_airspace_volume(2) 
        
        mock_callback1 = Mock(name="lease1_callback")
        mock_callback2 = Mock(name="lease2_callback")

        model.update_airspace(airspace=airspace1, callback=mock_callback1)
        name1 = model.target_airspace.name
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()

        mock_callback1.assert_not_called()
        mock_callback2.assert_not_called()

        model.update_airspace(airspace=airspace2, callback=mock_callback2)
        name2 = model.target_airspace.name
        assert name2 == 2
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        response1 = create_response(request_number=req_num1, current_lease=req_num1, approved=True, deadlock=False)
        model.receive_response(response1)

        self.assertEqual(model.airspace, airspace1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mock_callback1.assert_called_once_with(name1, airspace1, AirLeaseOutcome(True, False))
        mock_callback2.assert_not_called()
    
        self.assertNotIn(req_num1, model.pending_requests)
        self.assertIn(req_num2, model.pending_requests)
        self.assertEqual(model.target_airspace.name, name2)
        self.assertEqual(model.target_airspace.airspace, airspace2)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_receive_response_4(self, mock_next_request_number):
        """
        Test the case where:
        1. we ask for lease 1
        2. we ask for lease 2
        3. we ask for lease 3
        3. we receive a response for lease 2 (it's approved)

        expected behavior:
            - we should be in WAITING_RESPONSE
            - we removed the pending requests for leases 1 & 2
            - we should have a current lease of 2
            - we should have _NOT_ called the callback for lease 1
            - we should have called the callback for lease 2
            - we should have not called the callback for lease 3
        """

        mock_next_request_number.side_effect = [1000, 2000, 3000, 4000]
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertIsNone(model.airspace)

        airspace1 = create_airspace_volume(1) 
        airspace2 = create_airspace_volume(1)
        airspace3 = create_airspace_volume(1)

        
        callback1 = Mock(name="lease1_callback")
        callback2 = Mock(name="lease2_callback")
        callback3 = Mock(name="lease3_callback")

        model.update_airspace(airspace=airspace1, callback=callback1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.publish.call_args.kwargs['data']['request_number'] == 1000
        mqtt_gcs.reset_mock()
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_not_called()

        name2 = model.update_airspace(airspace=airspace2, callback=callback2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.publish.call_args.kwargs['data']['request_number'] == 2000
        mqtt_gcs.reset_mock()
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_not_called()

        name3 = model.update_airspace(airspace=airspace3, callback=callback3)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.publish.call_args.kwargs['data']['request_number'] == 3000
        mqtt_gcs.reset_mock()

        response = create_response(request_number=2000, current_lease=2000, approved=True, deadlock=False)
        model.receive_response(response)

        self.assertEqual(model.airspace, airspace2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        callback1.assert_not_called()
        callback2.called_once_with(name2, airspace2, AirLeaseOutcome(True, False))
        callback3.assert_not_called()

        self.assertNotIn(1000, model.pending_requests)
        self.assertNotIn(2000, model.pending_requests)
        self.assertIn(3000, model.pending_requests)
        self.assertEqual(model.target_airspace.airspace, airspace3)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_receive_response_5(self, mock_next_request_number):
        """
        Test the case where:
        1. we ask for lease 1
        2. we ask for lease 2
        3. we ask for lease 3
        4. we receive a response for lease 3 (it's approved)

        expected behavior:
            - we should be in IDLE
            - we removed the pending requests for leases 1, 2 & 3
            - we should have a current lease of 3
            - we should have _NOT_ called the callback for leases 1 & 2
            - we should have called the callback for lease 3
        """
        mock_next_request_number.side_effect = [1000 * i for i in range(1, 11)]
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        self.assertIsNone(model.airspace)

        airspace1 = create_airspace_volume(1) 
        airspace2 = create_airspace_volume(2)
        airspace3 = create_airspace_volume(1)

        
        callback1 = Mock(wraps=my_callback)
        callback2 = Mock(wraps=my_callback)
        callback3 = Mock(wraps=my_callback)

        model.update_airspace(airspace=airspace1, callback=callback1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_not_called()

        model.update_airspace(airspace=airspace2, callback=callback2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_not_called()

        name3 = model.update_airspace(airspace=airspace3, callback=callback3)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_not_called()

        response = create_response(request_number=3000, current_lease=3000, approved=True, deadlock=False)
        model.receive_response(response)
        # so we make sure we have the right state
        self.assertEqual(model.airspace, airspace3)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        callback1.assert_not_called()
        callback2.assert_not_called()
        callback3.assert_called_once_with(name3, airspace3, AirLeaseOutcome(True, False))
        self.assertNotIn(1000, model.pending_requests)
        self.assertNotIn(2000, model.pending_requests)
        self.assertNotIn(3000, model.pending_requests)

        self.assertEqual(model.target_airspace.airspace, airspace3)
        self.assertEqual(model.airspace, airspace3)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_tick_in_waiting_response_1(self, mock_next_request_number):
        """
        Test the retry logic when we don't get a response.

        Here's the scenario: we ask for an air lease. Then we wait for a
        response. But we don't get a response. So we keep waiting and waiting.
        Eventually we decide to ask for the airspace again. So we send a new
        request for the same airspace.

        This could happen when the request never reaches the ground station or
        the response never reaches the drone.

        The test proceeds as follows:
        1. The AirLeaserModel  sends an air lease request, and enters the WAITING_RESPONSE state.
        
        2. We `tick` the AirLeaserModel by 0.1 seconds to simulate the passage of time.

        3. After enough time has passed, the AirLeaserModel should send a new request for the same airspace. 
        
        4. The test verifies that the retry logic works correctly by checking that a new request is sent
        and that the system goes back to the WAITING_RESPONSE state.

        """
        all_req_numbers = [1000 * i for i in range(1, 11)]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace = create_airspace_volume(10)
        callback = Mock(wraps=my_callback)
        model.update_airspace(airspace=airspace, callback=callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        self.assertIn(1000, model.pending_requests)
        mqtt_gcs.reset_mock()
        callback.assert_not_called()

        # now we tick 2.0 seconds into the future
        for _ in range(20):
            mqtt_gcs.publish.assert_not_called()

            # Tick the model forward by 0.1 seconds
            # This is the main thing we're testing
            model.tick(0.1)
            
            callback.assert_not_called()

        # make sure we sent the request again
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        callback.assert_not_called()
        # look at our pending requests
        self.assertIn(1000, model.pending_requests)
        self.assertIn(2000, model.pending_requests)

        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_not_called()
        callback.assert_not_called()

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_tick_in_waiting_response_2(self, mock_next_request_number):
        """
        In this test we want to send a request, wait, and retry 100 times.
        """
        
        # we will need 100 request numbers
        all_req_numbers = [1000 * i for i in range(1, 102)] 
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace = create_airspace_volume(1)
        callback = Mock(wraps=my_callback)
        model.update_airspace(airspace=airspace, callback=callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        self.assertIn(1000, model.pending_requests)
        mqtt_gcs.reset_mock()
        callback.assert_not_called()

        # now we tick 200 seconds into the future going 0.1 seconds at a time
        for _ in range(20 * 100):
            # Tick the model forward by 0.1 seconds
            # This is the main thing we're testing
            model.tick(0.1)

            self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        # make sure we sent the request 100 times
        mqtt_gcs.publish.assert_called()
        self.assertEqual(mqtt_gcs.publish.call_count, 100)

        callback.assert_not_called()
        # look at our pending requests
        for req_num in all_req_numbers:
            self.assertIn(req_num, model.pending_requests)

        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        callback.assert_not_called()
    
    @patch("dr_onboard_autonomy.airlease.protocol.random.uniform")
    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_multi_scenario_01(self, mock_next_request_number, mock_uniform):
        """
        In this test we will:

        1. Ask for an airspace 1 and Get it. (req 0) 1000
        2. Ask for airspace 2 and get no response. (req 1) 2000
        3. Ask for airspace 2 and get no response. (req 2) 3000
        4. Ask for airspace 2 again (req 3) 4000
        5. get a response to req3... we find out airspace2 was approved by req 1 
        6. try to ask for airspace 3 but find out we're disconnected (req 4) 5000
        7. tick until we're connected again
        8. ask for airspace 3 (req 5) 6000 
        9. get denied (response 6000, current lease 2000, approved False, deadlock False)
        10. make sure we're in LEASE_DENIED
        11. tick until we send again
        """

        all_req_numbers = [1000 * i for i in range(1, 10)]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace1 = create_airspace_volume(1)
        mock_callback1 = Mock(wraps=my_callback)
        
        name1 = model.update_airspace(airspace1, mock_callback1)
        
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        # we will tick forward 1.5 seconds
        for _ in range(15):
        
            model.tick(0.1)
        
        # make sure we didn't send another request
        mqtt_gcs.publish.assert_not_called()
        mock_callback1.assert_not_called()
        self.assertEqual(len(model.pending_requests), 1)
        # now we need to simulate a response for airspace1
        req0 = all_req_numbers[0]
        response1 = create_response(req0, req0, True, False)
        
        model.receive_response(response1)
        mock_callback1.assert_called_once_with(name1, airspace1, airleaser_protocol.AirLeaseOutcome(True, False))
        self.assertNotIn(req0, model.pending_requests)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)

        # we will tick forward 10 seconds
        for _ in range(100):

            model.tick(0.1)
        
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)


        # now let's ask for airspace2
        airspace2 = create_airspace_volume(2)
        mock_callback2 = Mock(wraps=my_callback)

        name2 = model.update_airspace(airspace2, mock_callback2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        # we will tick forward 2 seconds
        for _ in range(20):
            model.tick(0.1)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback2.assert_not_called()
        # we should have two pending requests. req1 and req2
        req1 = all_req_numbers[1]
        req2 = all_req_numbers[2]
        self.assertIn(req1, model.pending_requests)
        self.assertIn(req2, model.pending_requests)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        # now we ask for airspace2 again by ticking 2 more seconds
        for _ in range(20):
            model.tick(0.1)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback2.assert_not_called()
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        # make sure we have 3 pending requests: 1, 2, 3
        for req in all_req_numbers[1:4]:
            self.assertIn(req, model.pending_requests)
        
        # now we will simulate a response for req3
        # this response says req1 was approved
        # so this request is denied because we specifed the wrong current lease
        # when we asked for req3
        req3 = all_req_numbers[3]
        response1 = create_response(req3, req1, False, False)

        model.receive_response(response1)
        # so the callback should tell us our airspace was **approved** we are just finding out now from this deny response...
        mock_callback2.assert_called_once_with(name2, airspace2, airleaser_protocol.AirLeaseOutcome(True, False))
        # make sure we have no pending requests
        self.assertEqual(model.pending_requests, {})

        # let's tick forward 90 seconds
        for _ in range(900):
            model.tick(0.1)
        
        # time to ask for airspace3
        airspace3 = create_airspace_volume(3)
        mock_callback3 = Mock(wraps=my_callback)
        # before we update_airspace let's simulate being disconnected
        mqtt_gcs.is_connected.return_value = False

        name3 = model.update_airspace(airspace3, mock_callback3)

        self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        # now lets tick 3 times and make sure we're still disconnected
        for _ in range(3):
            model.tick(0.1)
            self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        
        # now we're connected again
        mqtt_gcs.is_connected.return_value = True
        model.tick(0.1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], all_req_numbers[5])
        self.assertEqual(call_args.kwargs['data']['requests'][0]['current_lease'], req1)

        mqtt_gcs.reset_mock()
        mock_callback3.assert_not_called()
        model.tick(0.1)
        model.tick(0.1)
        # now we simulate a response for the most recent request
        req5 = all_req_numbers[5]
        response5 = create_response(req5, req1, False, True)
        # before we provide the response we need to mock the return value of mock_uniform
        mock_uniform.return_value = 1.05 # 11 ticks at 0.1 seconds per tick
        model.receive_response(response5)
        self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        # make sure we have no pending requests
        self.assertEqual(model.pending_requests, {})
        mock_callback3.assert_called_once_with(name3, airspace3, airleaser_protocol.AirLeaseOutcome(False, True))
        mock_callback3.reset_mock()
        self.assertEqual(model.airspace, airspace2)
        # now we tick forward 10 seconds
        for _ in range(10):
            model.tick(0.1)
            mqtt_gcs.publish.assert_not_called()
            mock_callback3.assert_not_called()
            self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        
        # now we tick 1 more time
        model.tick(0.1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], all_req_numbers[6])
        self.assertEqual(call_args.kwargs['data']['requests'][0]['current_lease'], req1)
        mqtt_gcs.reset_mock()
        mock_callback3.assert_not_called()

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_we_can_go_from_waiting_to_disconnected_to_idle(self, mock_next_request_number):
        """
        This tests an edge case.
        we send a request
        the connection goes down
        But we don't know it yet, so we keep waiting for a response
        then we try and send a new request but we find out we're disconnected

        Then we tick trying to discover if we're connected again
        before we discover that we are connected we get connected an a response that approves the original request arrives
        so we should go from DISCONNECTED directly to IDLE
        and model.airspace should be the airspace that was approved
        This is a test to make sure we can go from WAITING_RESPONSE to DISCONNECTED to IDLE
        """
        all_req_numbers = [1000 * i for i in range(1, 10)]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace = create_airspace_volume(3)
        mock_callback = Mock(wraps=my_callback)
        name = model.update_airspace(airspace, mock_callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback.assert_not_called()

        self.assertIn(all_req_numbers[0], model.pending_requests)
        self.assertEqual(len(model.pending_requests), 1)

        mqtt_gcs.is_connected.return_value = False
        # we will tick forward 2 seconds
        for _ in range(20): # tick 20 times to trigger the retry logic
            model.tick(0.1)
            mqtt_gcs.publish.assert_not_called()

        self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        self.assertIn(all_req_numbers[0], model.pending_requests)
        self.assertEqual(len(model.pending_requests), 1)
        mock_callback.assert_not_called()
        # make sure we called mock_next_request_number twice
        self.assertEqual(mock_next_request_number.call_count, 2)

        # now that we're disconnected we will tick forward 10 seconds
        for _ in range(100):
            model.tick(0.1)
            mqtt_gcs.publish.assert_not_called()
            mock_callback.assert_not_called()
            self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        
        # now we will restore the connection
        mqtt_gcs.is_connected.return_value = True
        # and we will provide a response for the original request
        response = create_response(all_req_numbers[0], all_req_numbers[0], True, False)
        
        model.receive_response(response)
        
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        # make sure model.airspace is correct
        self.assertEqual(model.airspace, airspace)
        # make sure the callback was called with the right args
        mock_callback.assert_called_once_with(name, airspace, airleaser_protocol.AirLeaseOutcome(True, False))

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_we_can_go_from_waiting_to_denied_to_idle(self, mock_next_request_number):
        """
        This tests another edge case.
        we send a request
        we don't get a response
        we send a new request
        we get a response that denies the original request

        Then we tick in the lease denied state
        but while we're in this state we get a response to the second request that approves the airspace
        so we need to go from LEASE_DENIED to IDLE
        and model.airspace should be the airspace that was approved 
        We need to make sure the callback is called when we're denied and when we are approved
        """
        all_req_numbers = [1000 * i for i in range(1, 10)]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace = create_airspace_volume(1)
        mock_callback = Mock(wraps=my_callback)

        name = model.update_airspace(airspace, mock_callback)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback.assert_not_called()

        # tick 2.1 seconds all at once
        model.tick(2.1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback.assert_not_called()

        # Now we need a response to the first request
        req0 = all_req_numbers[0]
        response = create_response(req0, 0, False, False)

        model.receive_response(response)
        self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        mock_callback.assert_called_once_with(name, airspace, airleaser_protocol.AirLeaseOutcome(False, False))
        mock_callback.reset_mock()
        self.assertEqual(model.airspace, None)
        self.assertNotIn(req0, model.pending_requests)
        self.assertIn(all_req_numbers[1], model.pending_requests)

        # now we tick a few times and approve the second request
        for _ in range(10):
            model.tick(0.1)
            mock_callback.assert_not_called()
            self.assertEqual(model.state, airleaser_protocol.States.LEASE_DENIED)
        
        response2 = create_response(all_req_numbers[1], all_req_numbers[1], True, False)
        model.receive_response(response2)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        mock_callback.assert_called_once_with(name, airspace, airleaser_protocol.AirLeaseOutcome(True, False))
        mock_callback.reset_mock()
        self.assertEqual(model.airspace, airspace)
        self.assertNotIn(all_req_numbers[1], model.pending_requests)
        # make sure pending is empty
        self.assertEqual(model.pending_requests, {})

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_we_can_restart_gcs_and_the_drone_accepts_current_lease_0(self, mock_next_request_number):
        """
        Then we will send another multi-tunnel request. We will deny it and specify the current_lease = 0
        This is to simulate what would happen if we restarted the GCS but not the drone
        We will send a multi-tunnel request. We will approve it.
        """
        all_req_numbers = [1000 * i for i in range(1, 10)]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        airspace = create_airspace_volume(3)
        mock_callback = Mock(wraps=my_callback)
        name = model.update_airspace(airspace, mock_callback)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback.assert_not_called()

        response = create_response(all_req_numbers[0], all_req_numbers[0], True, False)
        model.receive_response(response)
        mock_callback.assert_called_once_with(name, airspace, airleaser_protocol.AirLeaseOutcome(True, False))
        self.assertEqual(model.lease_id, all_req_numbers[0])
        
        # now we ask for a new airspace volume
        airspace2 = create_airspace_volume(2)
        mock_callback2 = Mock(wraps=my_callback)
        name2 = model.update_airspace(airspace2, mock_callback2)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback2.assert_not_called()
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

        # now we make a response that says we have lease ID 0
        response2 = create_response(all_req_numbers[1], 0, False, False)
        model.receive_response(response2)
        self.assertEqual(model.lease_id, 0)

    @patch("dr_onboard_autonomy.airlease.protocol.next_request_number")
    def test_that_we_update_lease_data_in_edge_case(self, mock_next_request_number):
        """
        This test is to make sure we update the airspace and lease ID when the following happens:

        1. we ask for airspace 1 (r1_0) (note r1_0 means request for airspace 1, request number 0)
        2. get a response for r1_0. It says it was approved
        3. we ask for airspace 2 but no response so we ask 2 more times (ultimately we send: r2_0, r2_1, r2_2)
        4. ask for airspace 3 but no response so we ask 2 more times (we end up sending: r3_0, r3_1, r3_2)
        5. we get a response to request r3_0. It says our current lease is r2_0. So we know r2_0 was approved but the response was lost

        Now we need to make sure that:
        - the lease_id is updated to r2_0
        - the airspace is updated to airspace 2
        - the pending requests are cleaned up (all requests up to r3_0 should be removed)

        Bonus steps!

        6. get a response for r2_0. It says it was approved (make sure nothing changes)
        7. get a response for r2_1. It says it was approved (make sure nothing changes)
        """
        all_req_numbers = [1000 * i for i in range(1, 10)]
        # gonna specify all request numbers we expect to use upfront for the rest of the test
        r1_0 = all_req_numbers[0]
        r2_0 = all_req_numbers[1]
        r2_1 = all_req_numbers[2]
        r2_2 = all_req_numbers[3]
        r3_0 = all_req_numbers[4]
        r3_1 = all_req_numbers[5]
        r3_2 = all_req_numbers[6]
        mock_next_request_number.side_effect = all_req_numbers

        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)

        # step 1 ask for airpsace 
        airspace1 = create_airspace_volume(1)
        mock_callback1 = Mock(wraps=my_callback)
        name1 = model.update_airspace(airspace1, mock_callback1)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        mqtt_gcs.reset_mock()
        mock_callback1.assert_not_called()

        # step 2 get response for airspace 1 r1_0
        response_r1_0 = create_response(r1_0, r1_0, True, False)
        model.receive_response(response_r1_0)
        mock_callback1.assert_called_once_with(name1, airspace1, airleaser_protocol.AirLeaseOutcome(True, False))
        self.assertEqual(model.lease_id, r1_0)
        self.assertEqual(model.airspace, airspace1)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        mock_callback1.reset_mock()

        # now for step 3 ask for airspace 2 and get no response so we ask 2 more times
        airspace2 = create_airspace_volume(2)
        mock_callback2 = Mock(wraps=my_callback)
        name2 = model.update_airspace(airspace2, mock_callback2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], r2_0)
        mqtt_gcs.reset_mock()
        mock_callback2.assert_not_called()

        # ask two more times...
        for r in [r2_1, r2_2]:
            model.tick(2.001)
            mqtt_gcs.publish.assert_called_once()
            call_args = mqtt_gcs.publish.call_args
            self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], r)
            mqtt_gcs.reset_mock()
            mock_callback1.assert_not_called()
            mock_callback2.assert_not_called()
            # make sure our current airspace is still airspace1
            self.assertEqual(model.airspace, airspace1)
            self.assertEqual(model.lease_id, r1_0)
        
        # now for step 4 ask for airspace 3 a total of 3 times
        airspace3 = create_airspace_volume(3)
        mock_callback3 = Mock(wraps=my_callback)
        name3 = model.update_airspace(airspace3, mock_callback3)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        call_args = mqtt_gcs.publish.call_args
        self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], r3_0)
        mqtt_gcs.reset_mock()
        mock_callback1.assert_not_called()
        mock_callback2.assert_not_called()
        mock_callback3.assert_not_called()

        for r in [r3_1, r3_2]: # ask two more times for airspace3
            model.tick(20.01) # 20.01 seconds is greater than the retry time of 2.0 seconds but we should still only send one request
            mqtt_gcs.publish.assert_called_once()
            call_args = mqtt_gcs.publish.call_args
            self.assertEqual(call_args.kwargs['data']['requests'][0]['request_number'], r)
            mqtt_gcs.reset_mock()
            mock_callback1.assert_not_called()
            mock_callback2.assert_not_called()
            mock_callback3.assert_not_called()
            # make sure our current airspace is still airspace1
            self.assertEqual(model.airspace, airspace1)
            self.assertEqual(model.lease_id, r1_0)

        # now step 5 we get a response for r3_0 that says our current lease is r2_0
        response_r3_0 = create_response(r3_0, r2_0, False, False) # must be denied because we specified the wrong current lease!
        model.receive_response(response_r3_0)
        mock_callback2.assert_called_once_with(name2, airspace2, airleaser_protocol.AirLeaseOutcome(True, False))
        self.assertEqual(model.lease_id, r2_0)
        self.assertEqual(model.airspace, airspace2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mock_callback1.assert_not_called()
        mock_callback2.reset_mock()
        mock_callback3.assert_not_called()
        # make sure we cleaned up the correct pending requests
        for r in [r1_0, r2_0, r2_1, r2_2, r3_0]:
            self.assertNotIn(r, model.pending_requests)
        self.assertIn(r3_1, model.pending_requests)
        self.assertIn(r3_2, model.pending_requests)

        # bonus step 6 get a response for r2_0 that says it was approved
        response_r2_0 = create_response(r2_0, r2_0, True, False)
        model.receive_response(response_r2_0)
        mock_callback1.assert_not_called()        
        mock_callback2.assert_not_called()
        mock_callback3.assert_not_called()
        self.assertEqual(model.lease_id, r2_0)
        self.assertEqual(model.airspace, airspace2)
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)

    def test_edge_case_discovered_in_field_july_9_2025(self):
        """Test to capture edge case discovered in the field.

        For the original issue see:
        https://github.com/DroneResponse/DR-OnboardAutonomy/issues/496

        (the issue includes logs and a description of the problem)
        
        This test reproduces a scenario where:
        0. Drone starts with an existing air lease
        1. Calls request_airspace() while MQTT is disconnected
        2. MQTT reconnects after 1-2 seconds
        3. Over ~11 seconds, multiple requests get buffered (sent every 2 seconds)
        4. First request gets approved
        5. Second request gets denied (wrong current_lease)
        6. Model incorrectly sends a "corrected" request
        7. Corrected request gets approved, leading to impossible state
        """
        # Setup: Initialize AirLeaserModel using the standard pattern
        mqtt_gcs = mock_gcs_mqtt()
        model, machine = airleaser_protocol.make_model_and_machine("TEST DRONE", mqtt_gcs)
        
        # Setup: Initialize ground service helper
        ground_service = AirLeaseGroundServiceHelper("TEST DRONE")
        
        # Step 0: Establish an initial air lease
        initial_airspace = create_airspace_volume(1)
        initial_callback = Mock(wraps=my_callback)
        initial_name = model.update_airspace(initial_airspace, initial_callback)
        
        # Process initial request through ground service
        self.assertEqual(model.state, airleaser_protocol.States.WAITING_RESPONSE)
        mqtt_gcs.publish.assert_called_once()
        initial_call_args = mqtt_gcs.publish.call_args
        initial_request_data = initial_call_args.kwargs['data']
        
        # Get response from ground service and feed it back
        initial_response_data = ground_service.process_request(initial_request_data)
        self.assertTrue(initial_response_data['approved'])
        initial_mqtt_response = mock_types.mock_mqtt_message(
            msg_sender_name="airlease_status",
            topic=f"drone/TEST DRONE/airlease/status",
            payload=json.dumps(initial_response_data)
        )['data']
        
        model.receive_response(initial_mqtt_response)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        initial_lease_id = initial_response_data['current_lease']
        self.assertEqual(model.lease_id, initial_lease_id)
        mqtt_gcs.reset_mock()
        
        # Step 1: Simulate MQTT disconnection and call request_airspace()
        mqtt_gcs.is_connected.return_value = False
        
        problem_airspace = create_airspace_volume(2)
        problem_callback = Mock(wraps=my_callback)
        problem_name = model.update_airspace(problem_airspace, problem_callback)
        
        # Should go to DISCONNECTED state since MQTT is down
        self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        mqtt_gcs.publish.assert_not_called()  # No request sent while disconnected
        
        # Step 2: Simulate 1-2 seconds passing, then reconnect MQTT
        for _ in range(15):  # 1.5 seconds at 0.1s per tick
            model.tick(0.1)
            self.assertEqual(model.state, airleaser_protocol.States.DISCONNECTED)
        
        # Reconnect MQTT
        mqtt_gcs.is_connected.return_value = True
        
        # Step 3: Simulate ~11 seconds of ticking to capture buffered requests
        buffered_requests = []
        
        for tick_count in range(110):  # 11 seconds at 0.1s per tick
            model.tick(0.1)
            
            # Check if a new request was sent (every ~2 seconds)
            if mqtt_gcs.publish.called and len(buffered_requests) < 6:
                call_args = mqtt_gcs.publish.call_args
                request_data = call_args.kwargs['data']
                buffered_requests.append(request_data.copy())
                mqtt_gcs.reset_mock()
        
        # Should have captured 5-6 buffered requests
        self.assertGreaterEqual(len(buffered_requests), 5)
        self.assertLessEqual(len(buffered_requests), 6)
        print(f"Captured {len(buffered_requests)} buffered requests")
        
        # Step 4: Process first request - should be approved
        first_request = buffered_requests[0]
        first_response_data = ground_service.process_request(first_request)
        self.assertTrue(first_response_data['approved'])
        
        first_mqtt_response = mock_types.mock_mqtt_message(
            msg_sender_name="airlease_status",
            topic=f"drone/TEST DRONE/airlease/status",
            payload=json.dumps(first_response_data)
        )['data']
        
        model.receive_response(first_mqtt_response)
        self.assertEqual(model.state, airleaser_protocol.States.IDLE)
        
        # Step 5: Process second request - should be denied (wrong current_lease)
        second_request = buffered_requests[1]
        second_response_data = ground_service.process_request(second_request)
        self.assertFalse(second_response_data['approved'])  # Should be denied
        
        second_mqtt_response = mock_types.mock_mqtt_message(
            msg_sender_name="airlease_status",
            topic=f"drone/TEST DRONE/airlease/status",
            payload=json.dumps(second_response_data)
        )['data']
        
        # This should trigger the problematic behavior
        model.receive_response(second_mqtt_response)
        
        # Step 6: Check if model sends a "corrected" request (this is the bug)
        if mqtt_gcs.publish.called:
            corrected_call_args = mqtt_gcs.publish.call_args
            corrected_request_data = corrected_call_args.kwargs['data']
            
            # Process corrected request - should be approved
            corrected_response_data = ground_service.process_request(corrected_request_data)
            self.assertTrue(corrected_response_data['approved'])
            
            corrected_mqtt_response = mock_types.mock_mqtt_message(
                msg_sender_name="airlease_status",
                topic=f"drone/TEST DRONE/airlease/status",
                payload=json.dumps(corrected_response_data)
            )['data']
            
            # This should lead to the impossible state
            try:
                model.receive_response(corrected_mqtt_response)
                print("Model processed corrected response without crashing")
            except Exception as e:
                print(f"Model crashed with exception: {e}")
                # This might be the impossible state we're looking for
        
        # Verify final state
        print(f"Final model state: {model.state}")
        print(f"Final lease_id: {model.lease_id}")
        print(f"Pending requests: {len(model.pending_requests)}")
        
        # TODO: Add assertions to verify the impossible state condition


