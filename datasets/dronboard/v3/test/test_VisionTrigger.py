import inspect
import math
import os
import pathlib
import json
from typing import Any, List
import unittest
from unittest.mock import NonCallableMock, call, patch

from dr_onboard_autonomy.models.config import CameraConfig
from tf.transformations import (
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
)
from droneresponse_mathtools import Lla
from pyquaternion import Quaternion as Quat

from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition, Quaternion
from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import VisionTrigger, Gimbal
from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator
from dr_onboard_autonomy.gimbal.geolocation import GimbalLimits, TwoAxisGimbalQuatCalculator


from test.mock_types import mock_copter_drone, mock_mqtt_message, mock_state_kwargs, mock_message

import inspect

VisionTrigger_module = inspect.getmodule(VisionTrigger)

def mock_message_senders():
    vision_found_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    vision_found_msg_sender.name = "vision"

    vision_request_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    vision_request_msg_sender.name = "vision_request"

    target_position_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    target_position_msg_sender.name = "target_position"

    def find(name):
        if name == "vision_found":
            return vision_found_msg_sender
        if name == "vision_request":
            return vision_request_msg_sender
        if name == "target_position":
            return target_position_msg_sender
        else:
            msg_sender = NonCallableMock(spec=AbstractMessageSender)
            return msg_sender

    reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
    reusable_message_senders.find.side_effect = find
    return reusable_message_senders


def mock_state(outcomes_to_add: List[str]=[]):
    outcomes = ["success"]
    outcomes.extend(outcomes_to_add)

    kwargs = mock_state_kwargs(
        dict(
            outcomes=outcomes,
            air_lease_service=NonCallableMock(spec=AirLeaseService),
            drone=mock_copter_drone(),
            heartbeat_handler=False,
            local_mqtt_client=NonCallableMock(spec=MQTTClient),
            mqtt_client=NonCallableMock(spec=MQTTClient),
            name="test_state",
            reusable_message_senders=mock_message_senders(),
            trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
        )
    )

    state = BaseState(**kwargs)
    return state

def mock_state_dr_cube(outcomes_to_add: List[str]=[]):
    """Create a mock state that matches one that the dr_cube would build.
    
    This will have
    - a TwoAxisGimbalQuatCalculator with a roll first config and gimbal limits that match the mio
    - a 74 degree horizontal fov, 42 degree vertical fov, 
    """
    outcomes = ["success"]
    outcomes.extend(outcomes_to_add)

    gimbal_calc = TwoAxisGimbalQuatCalculator(
        "roll_first",
        GimbalLimits(-60,60),
        GimbalLimits(-127,127),
    )

    kwargs = mock_state_kwargs(
        dict(
            outcomes=outcomes,
            air_lease_service=NonCallableMock(spec=AirLeaseService),
            drone=mock_copter_drone(),
            heartbeat_handler=False,
            local_mqtt_client=NonCallableMock(spec=MQTTClient),
            mqtt_client=NonCallableMock(spec=MQTTClient),
            name="test_state",
            reusable_message_senders=mock_message_senders(),
            trajectory_class=NonCallableMock(spec=TrajectoryGenerator),
            gimbal_calculator=gimbal_calc,
            camera_config=CameraConfig(horizontal_fov=74.0, vertical_fov=42.0),
        )
    )

    state = BaseState(**kwargs)
    return state

def mock_gimbal_driver():
    gimbal_driver = NonCallableMock(spec=Gimbal)
    return gimbal_driver


def mock_message(type: str, data: Any):
    return {
        "type": type,
        "data": data
    }

def mock_vision_message():
    payload = """
    {
        "x": 0.5,
        "y": 0.5,
        "ts": 1,
        "x_res": 416,
        "y_res": 416,
    }
    """
    mqtt_message = NonCallableMock()
    mqtt_message.payload = payload.encode("utf-8")
    mqtt_message.topic = "'dr-onboard/found'"
    return mock_message("vision_found", mqtt_message)

def mock_target_position_message():
    payload = """
    {
        "latitude": 39.747394,
        "longitude": -105.044845,
        "altitude_amsl": 1578.56
    }
    """
    mqtt_message = NonCallableMock()
    mqtt_message.payload = payload.encode("utf-8")
    mqtt_message.topic = "target_position"
    return mock_message("target_position", mqtt_message)


def add_drone_data(t, state: BaseState):
    """Add drone data to the state at time t
    This just writes some made-up data to the state's drone data

    We use this when the time of the drone data is important, not the actual data
    """
    drone_pos = LlaPosition(39.747394, -105.044845, 1578.56, DATUM_REFERENCE.ELLIPSOID_WGS84)
    drone_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)
    quaternion_about_axis(angle=math.radians(90), axis=[0, 1, 0])
    gimbal_attitude = Quaternion(0.0, 0.0, 0.0, 1.0)

    state.drone.data.location.add_position(t, drone_pos)
    state.drone.data.attitude.add_attitude(t, drone_attitude)
    state.drone.gimbal.attitude.add_attitude(t, gimbal_attitude)


def read_camera_params(file_path):
    this_dir = os.path.dirname(os.path.realpath(__file__))
    data_dir = pathlib.Path(this_dir) / "data" / "geo_locate_params" / file_path
    with open(data_dir, "r") as f:
        return json.load(f)


class TestVisionTrigger(unittest.TestCase):
    def test_the_init_method(self):
        state = mock_state(outcomes_to_add=["found"])
        self.assertIsNotNone(state.vision_trigger)
        self.assertEqual(type(state.vision_trigger), VisionTrigger)
    
    def test_on_request_sends_a_response(self):
        """Test that the on_request method sends a response to the local mqtt client
        """
        state = mock_state(outcomes_to_add=["found"])
        timestamp_ms = 1726081758183 # milliseconds
        t0 = timestamp_ms / 1000 - 3 # 3 seconds before the request (in seconds)
        t1 = timestamp_ms / 1000 + 3 # 3 seconds after the request (in seconds)

        lla0 = LlaPosition(39.747394, -105.044845, 1578.56, DATUM_REFERENCE.ELLIPSOID_WGS84)
        lla1 = LlaPosition(39.747394, -105.044845, 1578.56, DATUM_REFERENCE.ELLIPSOID_WGS84)
        state.drone.data.location.add_position(t0, lla0)
        state.drone.data.location.add_position(t1, lla1)

        drone_quat = Quaternion(0.0, 0.0, 0.0, 1.0)
        state.drone.data.attitude.add_attitude(t0, drone_quat)
        state.drone.data.attitude.add_attitude(t1, drone_quat)

        state.drone.gimbal.attitude.add_attitude(t0, drone_quat)
        state.drone.gimbal.attitude.add_attitude(t1, drone_quat)
        payload = json.dumps({
            "timestamp": timestamp_ms,
            "x": 100,
            "y": 800,
            "x_res": 1920,
            "y_res": 1080,
        })
        msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)
        state.vision_trigger.on_request(msg)
        state.local_mqtt_client.publish.assert_called()
    
    def test_on_request_sends_a_response_in_error_case01(self):
        """Test that the on_request method sends a response to the local mqtt client
        """
        state = mock_state(outcomes_to_add=["found"])
        timestamp_ms = 1726081758183 # milliseconds
        t0 = timestamp_ms / 1000 - 3 # 3 seconds before the request (in seconds)
        t1 = timestamp_ms / 1000 + 3 # 3 seconds after the request (in seconds)

        lla0 = LlaPosition(39.747394, -105.044845, 1578.56, DATUM_REFERENCE.ELLIPSOID_WGS84)
        lla1 = LlaPosition(39.747394, -105.044845, 1578.56, DATUM_REFERENCE.ELLIPSOID_WGS84)
        state.drone.data.location.add_position(t0, lla0)
        state.drone.data.location.add_position(t1, lla1)

        drone_quat = Quaternion(0.0, 0.0, 0.0, 1.0)
        state.drone.data.attitude.add_attitude(t0, drone_quat)
        state.drone.data.attitude.add_attitude(t1, drone_quat)

        state.drone.gimbal.attitude.add_attitude(t0, drone_quat)
        state.drone.gimbal.attitude.add_attitude(t1, drone_quat)
        # test that it sends the error message when we forget to add a timestamp
        payload = json.dumps({
            # "timestamp": timestamp_ms,
            "x": 100,
            "y": 800,
            "x_res": 1920,
            "y_res": 1080,
        })
        msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)
        state.vision_trigger.on_request(msg)
        state.local_mqtt_client.publish.assert_called()
        # look at the args of the first call
        args, kwargs = state.local_mqtt_client.publish.call_args
        msg = args[1]
        self.assertIn("success", msg)
        self.assertEqual(msg["success"], False)

    def test_the_init_method_no_found_outcome(self):
        state = mock_state()
        # so in this case, we should init a VisionTrigger object
        # but we should not respond to the "found" message
        # the easiest way to test this is to check that we don't return
        # an outcome when we process a "found" message
        msg = mock_message("found", {})
        outcome = state.handlers.notify(msg)
        self.assertIsNone(outcome)


    def test_vision_message(self):
        state = mock_state(outcomes_to_add=["found"])
        vision_msg = mock_vision_message()
        state.message_queue.put(vision_msg)
        outcome = state.execute(None)
        self.assertEqual(outcome, "found")

    def test_message_sender_was_added(self):
        state = mock_state(outcomes_to_add=["found"])

        state.reusable_message_senders.find.assert_has_calls([
            call("vision_found"),
            call("vision_request")
        ])

        vision_found_sender = state.reusable_message_senders.find("vision_found")
        self.assertIn(vision_found_sender, state.message_senders)

        vision_request = state.reusable_message_senders.find("vision_request")
        self.assertIn(vision_request, state.message_senders)
    
    def test_vision_messages_are_retried_if_needed(self):
        """Test that VisionTrigger retries vision messages if we don't have enough data yet.

        The vision request should be saved to the request queue. Periodically, we should attempt to
        redo all the requests in the request queue.

        In this test we will simulate the following scenario:
        - we get some drone data at time t0 and t1
        - we get a vision request at time t2
            assert that we do not send a response
        - we attempt to reprocess the request
            assert that we do not send a response
        - we get more drone data at time t3
        """
        state = mock_state(outcomes_to_add=["found"])

        timestamp_ms = 1726081758183 # milliseconds
        t0 = timestamp_ms / 1000 - 2 # 2 seconds before the request (in seconds)
        t1 = timestamp_ms / 1000 - 0.5 # 0.5 seconds before the request (in seconds)
        t2 = timestamp_ms / 1000 # the time of the request
        t3 = timestamp_ms / 1000 + 3 # 3 seconds after the request (in seconds)

        add_drone_data(t0, state)
        add_drone_data(t1, state)

        # make the vision request message
        payload = json.dumps({
            "timestamp": timestamp_ms,
            "x": 100,
            "y": 800,
            "x_res": 1920,
            "y_res": 1080,
        })
        request_msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)

        state.handlers.notify(request_msg)
        # we should not send a response
        state.local_mqtt_client.publish.assert_not_called()
        
        # now let's simulate the batch processing of queued requests
        retry_event = mock_message("retry_vision_calculations", {})
        state.handlers.notify(retry_event)
        # state.vision_trigger.on_batch_process_requests()
        # since we still didn't add the missing data, we should not send a response
        state.local_mqtt_client.publish.assert_not_called()

        # now let's add the missing data
        add_drone_data(t3, state)
        # attempt to reprocess the request
        state.handlers.notify(retry_event)
        state.local_mqtt_client.publish.assert_called()

    def test_vision_messages_stops_retrying_after_3_attempts(self):
        """Test that VisionTrigger only reattempts vision requests 3 times max
        
        So in case we get a request and we don't have enough data yet, we should enqueue it retry the request

        However, we should only retry 3 times max. After that, we should send an error response

        NOTE the first time we see the request, doesn't count as a retry.
        
        After the first time, we should retry 3 times and see an error response

        So the message is given 4 chances to be processed...
        """
        state = mock_state(outcomes_to_add=["found"])

        timestamp_ms = 1726081758183 # milliseconds
        t0 = timestamp_ms / 1000 - 2 # 2 seconds before the request (in seconds)
        t1 = timestamp_ms / 1000 - 0.5 # 0.5 seconds before the request (in seconds)
        t2 = timestamp_ms / 1000 # the time of the request
        t3 = timestamp_ms / 1000 + 3 # 3 seconds after the request (in seconds)

        add_drone_data(t0, state)
        add_drone_data(t1, state)

        # make the vision request message
        payload = json.dumps({
            "timestamp": timestamp_ms,
            "x": 100,
            "y": 800,
            "x_res": 1920,
            "y_res": 1080,
        })
        request_msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)

        # introduce the request to the system
        # this is the initial attempt (not a reattempt)
        # this doesn't count as one of the 3 retries
        state.handlers.notify(request_msg)
        state.local_mqtt_client.publish.assert_not_called() # we should not send a response because we don't have enough data
        
        # now let's simulate the retry event
        retry_msg = mock_message("retry_vision_calculations", {})
        
        # 1nd reattempt 
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()

        # 2nd reattempt
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()

        # this is the last time we should retry
        # since we still don't have data we need to 
        # and we expect to get an error response
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.asert_called()
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        self.assertEqual(topic, "vision/resp-location")

        msg = args[1]
        self.assertIn("success", msg)
        self.assertEqual(msg["success"], False)

    @patch.object(VisionTrigger_module.geolocation, "geolocate_object_from_camera")
    def test_vision_messages_sends_a_success_response_on_the_third_reattempt(self, geolocate_func):
        """Test that VisionTrigger sends a success response when it reattempts the vision request 3 times
        
        So in case we get a request and we don't have enough data yet, we should enqueue it, and retry the request

        However, we should only retry 3 times max. After that, we should send an error response.
        The first time we see the request, doesn't count as one of the three _retries_.
        After we receive the request and attempt it, we should retry up to  3 more times.
        
        In this test, before retrying for the last time we will add the missing data and make sure we send a success response
        """
        geolocate_func.return_value = [39.747394, -105.044845, 1578.56]
        state = mock_state(outcomes_to_add=["found"])

        timestamp_ms = 1726081758183 # milliseconds
        t0 = timestamp_ms / 1000 - 2 # 2 seconds before the request (in seconds)
        t1 = timestamp_ms / 1000 - 0.5 # 0.5 seconds before the request (in seconds)
        t2 = timestamp_ms / 1000 # the time of the request
        t3 = timestamp_ms / 1000 + 0.5 # 0.5 seconds after the request (in seconds)

        add_drone_data(t0, state)
        add_drone_data(t1, state)

        # make the vision request message
        payload = json.dumps({
            "timestamp": timestamp_ms,
            "x": 100,
            "y": 800,
            "x_res": 1920,
            "y_res": 1080,
        })
        request_msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)

        # introduce the request to the system
        # this is the initial attempt (not a reattempt)
        # this doesn't count as one of the 3 retries
        state.handlers.notify(request_msg)
        state.local_mqtt_client.publish.assert_not_called() # we should not send a response because we don't have enough data
        
        # now let's simulate the retry event
        retry_msg = mock_message("retry_vision_calculations", {})
        
        # 1nd reattempt 
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()

        # 2nd reattempt
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()

        # before we retry for the last time, let's add the missing data
        add_drone_data(t3, state)
        # now retry for the last time
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.asert_called()
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        self.assertEqual(topic, "vision/resp-location")

        msg = args[1]
        self.assertIn("success", msg)
        # this time the response should be a success
        self.assertEqual(msg["success"], True)
    
    def test_the_batch_process_method_when_there_are_no_messages(self):
        """Test that VisionTrigger does nothing when there are no messages to process
        """
        state = mock_state(outcomes_to_add=["found"])
        retry_msg = mock_message("retry_vision_calculations", {})
        outcome = state.handlers.notify(retry_msg)
        self.assertIsNone(outcome)
        state.local_mqtt_client.publish.assert_not_called()
    
    def test_the_batch_process_method_there_are_many_messages(self):
        """Test that VisionTrigger processes all the messages in the request queue

        so we will have data at t0, and t1
        """
        state = mock_state_dr_cube(outcomes_to_add=["found"])

        timestamp_ms = 1726081758183
        t0 = timestamp_ms / 1000 - 1.0
        t1 = timestamp_ms / 1000 - 0.5
        add_drone_data(t0, state)
        add_drone_data(t1, state)
        # now we will create 10 vision requests all 1ms apart
        vision_requests = []
        vision_timestamps = []
        for i in range(10):
            timestamp = timestamp_ms + i * 2
            payload = json.dumps({
                "timestamp": timestamp,
                "x": 100,
                "y": 800,
                "x_res": 1920,
                "y_res": 1080,
            })
            request_msg = mock_mqtt_message('vision_request', 'dr-onboard/req-location-from-frame-data', payload)
            vision_requests.append(request_msg)
            vision_timestamps.append(timestamp)
            state.handlers.notify(request_msg)
            state.local_mqtt_client.publish.assert_not_called()

        
        retry_msg = mock_message("retry_vision_calculations", {})
        # 1st retry attempt
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()

        # lets add half of the data we need
        t2 = vision_timestamps[4] + 1 # the 5th request
        t2 = t2 / 1000
        # note we made all the requests 2ms apart
        # so we can just add 1ms to the 5th request to create a clear separation
        add_drone_data(t2, state)
        # now lets run the batch process again
        # 2nd retry attempt
        state.handlers.notify(retry_msg)
        # make sure the first half of the requests were processed
        # we need to access all the times that publish was called
        # we expect to find 5 calls to publish
        self.assertEqual(state.local_mqtt_client.publish.call_count, 5)
        # now we need to check the timestamps of the calls
        publish_calls = state.local_mqtt_client.publish.call_args_list

        for ts, call in zip(vision_timestamps[:5], publish_calls):
            call_args, call_kwargs = call
            topic = call_args[0]
            msg = call_args[1]
            self.assertEqual(topic, "vision/resp-location")
            self.assertIn("success", msg)
            self.assertEqual(msg["success"], True)

        state.local_mqtt_client.publish.reset_mock()

        # before we run the batch process again, lets add data for half of the remaining requests
        # these requests remain: 5, 6, 7, 8, 9
        # lets add data for the 7th request
        t3 = vision_timestamps[6] + 1
        t3 = t3 / 1000
        add_drone_data(t3, state)

        # 3rd retry attempt
        state.handlers.notify(retry_msg)
        expected_times = vision_timestamps[5:]
        # for time stamps 5, 6, 7 we should have a success response
        success_timestamps = vision_timestamps[:7]

        publish_calls = state.local_mqtt_client.publish.call_args_list
        for ts, call in zip(expected_times, publish_calls):
            call_args, call_kwargs = call
            topic = call_args[0]
            msg = call_args[1]
            self.assertEqual(topic, "vision/resp-location")
            self.assertIn("success", msg)
            if ts in success_timestamps:
                self.assertEqual(msg["success"], True)
            else:
                self.assertEqual(msg["success"], False)
        state.local_mqtt_client.publish.reset_mock()

        # notify again
        # make sure we don't call publish again
        state.handlers.notify(retry_msg)
        state.local_mqtt_client.publish.assert_not_called()
        



class TestTargetPosition(unittest.TestCase):
    def test_TargetPosition_returns_target_position_outcome(self):
        state = mock_state(outcomes_to_add=["target_position"])

        state.message_queue.put(mock_target_position_message())
        outcome = state.execute(None)
        self.assertEqual(outcome, "target_position")

        target_position_saved = state.drone.data.target_position

        self.assertIsNotNone(target_position_saved)
        mock_target_pos_dict = json.loads(mock_target_position_message()["data"].payload)
        self.assertAlmostEqual(
            target_position_saved.latitude,
            mock_target_pos_dict["latitude"]
        )
        self.assertAlmostEqual(
            target_position_saved.longitude,
            mock_target_pos_dict["longitude"]
        )
        self.assertAlmostEqual(
            target_position_saved.altitude,
            mock_target_pos_dict["altitude_amsl"]
        )


    def test_message_sender_was_added(self):
        """Since reusable_message_senders is mocked, this test checks the TargetPosition component
        successfully adds the mocked target_position message sender to the state's message_senders
        """
        state = mock_state(outcomes_to_add=["target_position"])
        state.reusable_message_senders.find.assert_called_with("target_position")
        self.assertIn(
            state.reusable_message_senders.find("target_position"),
            state.message_senders
        )

    def test_end_to_end_test1(self):
        """Test that the VisionTrigger component can response to a Vision Service: Geo-location Response
        We will provide sensor data and a vision request, and verify the response.
        """

        drone_attitude_enu = Quat(angle=math.radians(40), axis=[0, 1, 0])
        camera_attitude_gimbal = Quat(angle=math.radians(-45), axis=[0, 1, 0])

        home = Lla(
            41.606673422437055, 
            -86.35559755369852,
            229.0,
        )

        drone = home.move_ned(0, -10, -10)
        camera_params = {
            "timestamp_ms": 1728529215230,
            "x": 50,
            "y": 50,
            "x_res": 100,
            "y_res": 100,
            "home_position": {
                "latitude": home.latitude, 
                "longitude": home.longitude,
                "altitude": home.altitude,
                "datum_ref": "ELLIPSOID_WGS84",
            },
            "drone_position": {
                "latitude": drone.latitude,
                "longitude": drone.longitude,
                "altitude": drone.altitude,
                "datum_ref": "ELLIPSOID_WGS84",
            },
            "drone_attitude_enu": {
                # pitch 45 degrees down
                "x": drone_attitude_enu.x,
                "y": drone_attitude_enu.y,
                "z": drone_attitude_enu.z,
                "w": drone_attitude_enu.w,
            },
            "gimbal_attitude_enu": None,
            "camera_attitude_gimbal": {
                "x": camera_attitude_gimbal.x,
                "y": camera_attitude_gimbal.y,
                "z": camera_attitude_gimbal.z,
                "w": camera_attitude_gimbal.w,
            },
            "camera_attitude_enu": None,
            "camera_config": {
                "horizontal_fov": 90,
                "vertical_fov": 90,
            },
            "res_topic": "vision/resp-location",
        }
        
        state = mock_state_dr_cube(outcomes_to_add=["found"])

        ## RUN THE GEO-LOCATION METHOD
        with patch.object(VisionTrigger_module.rospy, "loginfo") as loginfo:
            loginfo.side_effect = print
            state.vision_trigger.geo_locate_object_from_camera(camera_params)

        # make sure the response was sent
        state.local_mqtt_client.publish.assert_called()

        # check camera_attitude_enu
        camera_enu = VisionTrigger_module.Helpers.to_quat(camera_params['camera_attitude_enu'])
        camera_enu_expected = Quat(angle=math.radians(-45), axis=[0, 1, 0])
        # self.assertEqual(camera_enu, camera_enu_expected)
        # self.assertAlmostEqual(camera_enu.x, camera_enu_expected.x)
        # self.assertAlmostEqual(camera_enu.y, camera_enu_expected.y)
        # self.assertAlmostEqual(camera_enu.z, camera_enu_expected.z)
        # self.assertAlmostEqual(camera_enu.w, camera_enu_expected.w)
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        self.assertEqual(topic, "vision/resp-location")
        msg = args[1]
        self.assertIn("success", msg)
        self.assertEqual(msg["success"], True)
        self.assertIn("timestamp", msg)
        self.assertIn("lat", msg)
        self.assertIn("lon", msg)
        self.assertIn("alt_amsl", msg)

        # make sure timestamp is correct 
        self.assertEqual(msg["timestamp"], camera_params["timestamp_ms"])
        output_pos = LlaPosition(msg["lat"], msg["lon"], msg["alt_amsl"], datum_ref=DATUM_REFERENCE.AMSL)
        output_pos = output_pos.to_wgs84_ellipsoid()

        home = Lla(
            camera_params["home_position"]["latitude"],
            camera_params["home_position"]["longitude"],
            camera_params["home_position"]["altitude"]
        )

        actual_pos = Lla(
            output_pos.latitude,
            output_pos.longitude,
            output_pos.altitude
        )

        n, e, d = home.distance_ned(actual_pos)

        self.assertAlmostEqual(n, 0.0, places=2)
        self.assertAlmostEqual(e, 0.0, places=2)
        self.assertAlmostEqual(d, 0.0, places=2)
    
    def test_end_to_end_test2(self):
        params = read_camera_params("mock_peppermint_field_01.json")
        state = mock_state_dr_cube(outcomes_to_add=["found"])
        ## RUN THE GEO-LOCATION METHOD
        with patch.object(VisionTrigger_module.rospy, "loginfo") as loginfo:
            loginfo.side_effect = print
            state.vision_trigger.geo_locate_object_from_camera(params)
        # make sure the response was sent
        state.local_mqtt_client.publish.assert_called()
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        self.assertEqual(topic, "DIFFERENT/vision/resp-location")
        msg = args[1]
        self.assertEqual(msg["timestamp"], params["timestamp_ms"])
        self.assertEqual(msg["success"], True)
        
        object_lla = LlaPosition(
            msg["lat"],
            msg["lon"],
            msg["alt_amsl"],
            DATUM_REFERENCE.AMSL
        )
        object_lla = object_lla.to_wgs84_ellipsoid()
        object_lla = Lla(object_lla.latitude, object_lla.longitude, object_lla.altitude)

        expected_lla = Lla(
            params["home_position"]["latitude"],
            params["home_position"]["longitude"],
            params["home_position"]["altitude"]
        )

        n, e, d = expected_lla.distance_ned(object_lla)
        self.assertAlmostEqual(n, 0.0, places=2)
        self.assertAlmostEqual(e, 0.0, places=2)
        self.assertAlmostEqual(d, 0.0, places=2)
        
        assert expected_lla.distance(object_lla) < 0.001

    @unittest.skip
    def test_end_to_end_test3(self):
        params = read_camera_params("peppermint_field_2024-10-10.json")
        state = mock_state_dr_cube(outcomes_to_add=["found"])
        ## RUN THE GEO-LOCATION METHOD
        with patch.object(VisionTrigger_module.rospy, "loginfo") as loginfo:
            loginfo.side_effect = print
            state.vision_trigger.geo_locate_object_from_camera(params)
        # make sure the response was sent
        state.local_mqtt_client.publish.assert_called()
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        self.assertEqual(topic, "DIFFERENT/vision/resp-location")
        msg = args[1]
        self.assertEqual(msg["timestamp"], params["timestamp_ms"])
        self.assertEqual(msg["success"], True)
        
        object_lla = LlaPosition(
            msg["lat"],
            msg["lon"],
            msg["alt_amsl"],
            DATUM_REFERENCE.AMSL
        )
        object_lla = object_lla.to_wgs84_ellipsoid()
        object_lla = Lla(object_lla.latitude, object_lla.longitude, object_lla.altitude)

        expected_lla = Lla(
            params["expected_position"]["latitude"],
            params["expected_position"]["longitude"],
            params["expected_position"]["altitude"]
        )

        n, e, d = expected_lla.distance_ned(object_lla)
        self.assertAlmostEqual(n, 0.0, places=2)
        self.assertAlmostEqual(e, 0.0, places=2)
        self.assertAlmostEqual(d, 0.0, places=2)
        
        assert expected_lla.distance(object_lla) < 0.001
        
    def test_hitl_params_peppermint_field_test(self):
        """The drone was hovering over the shed at peppermint field.
        The gimbal was looking straight down.

        We captured the geo-location parameters for this scenario.
        """
        params = read_camera_params("hitl-test-shed-2024-10-24.json")
        state = mock_state_dr_cube(outcomes_to_add=["found"])
        ## RUN THE GEO-LOCATION METHOD
        with patch.object(VisionTrigger_module.rospy, "loginfo") as loginfo:
            loginfo.side_effect = print
            state.vision_trigger.geo_locate_object_from_camera(params)
        # Look at the response
        args, kwargs = state.local_mqtt_client.publish.call_args
        topic = args[0]
        message = args[1]

        # for this test we really care about the message. Make sure it's successful
        self.assertEqual(message["success"], True)
        print(message)


# Next our goal is to bring the DoubleBufferQueue class into the test scope
# We can do this by accessing the module in which the VisionTrigger class is defined
# However, the VisionTrigger module is shadowed by the VisionTrigger class
# So we need to get the module in a different way:
VisionTrigger_module = inspect.getmodule(VisionTrigger)


# Now we can access the DoubleBufferQueue class
DoubleBufferQueue = VisionTrigger_module.DoubleBufferQueue


class TestDoubleBufferQueue(unittest.TestCase):
    def test_init(self):
        dbq = DoubleBufferQueue()
        self.assertIsNotNone(dbq)
        self.assertIsInstance(dbq, DoubleBufferQueue)
    
    def test_put(self):
        dbq = DoubleBufferQueue()
        dbq.put(1)
        # make sure the read buffer is empty
        self.assertTrue(dbq.empty())
    
    def test_put_swap_read(self):
        dbq = DoubleBufferQueue()
        dbq.put(1)
        self.assertTrue(dbq.empty())
        dbq.swap()
        self.assertFalse(dbq.empty())
        self.assertEqual(dbq.get(), 1)
        self.assertTrue(dbq.empty())
    
    def test_swap_empty(self):
        dbq = DoubleBufferQueue()
        self.assertTrue(dbq.empty())
        dbq.swap()
        self.assertTrue(dbq.empty())
    
    def big_write_big_read(self):
        dbq = DoubleBufferQueue()

        for _ in range(10):
            self.assertTrue(dbq.empty())
            for i in range(1000):
                dbq.put(i)
                self.assertTrue(dbq.empty())
            dbq.swap()
            for i in range(1000):
                self.assertFalse(dbq.empty())
                self.assertEqual(dbq.get(), i)
            self.assertTrue(dbq.empty())
    
        self.assertTrue(dbq.empty())
        dbq.swap()
        self.assertTrue(dbq.empty())
        
