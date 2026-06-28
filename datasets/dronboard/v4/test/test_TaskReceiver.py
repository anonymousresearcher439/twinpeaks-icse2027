import importlib
import json
import queue
import sys
import time
import unittest

from unittest.mock import patch, NonCallableMock, Mock
from dr_onboard_autonomy.models.kinematics import LlaPosition, DATUM_REFERENCE
from dr_onboard_autonomy.briar_helpers import BriarLla

from dr_onboard_autonomy.states import RunTasks, TaskReceiver
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory

from . import mock_types


RunTasks_module = importlib.import_module(RunTasks.__module__)

READY_TIMER_NAME = TaskReceiver.TIMER_MESSAGE_NAME


def mock_task_receiver(outcome, task_dict):
    task_receiver = mock_types.mock_state(outcome, TaskReceiver)
    task_json = json.dumps(task_dict)
    task_receiver.get.return_value = task_json.encode("utf-8")
    return task_receiver


def add_position_data_to_drone(kwargs):
    home = kwargs["home"]
    
    pos = BriarLla.from_dict(home).ellipsoid.lla
    pos = LlaPosition(pos.latitude, pos.longitude, pos.altitude, datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84)
    t = 1708640881.7664595
    drone = kwargs["drone"]
    drone.data.location.add_position(t, pos)



class TestTaskReceiver(unittest.TestCase):
    def test_init(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        state = TaskReceiver(**kwargs)
        
        self.assertIsInstance(state.trajectory, HoldingTrajectory)
        self.assertIsInstance(state, TaskReceiver)
    
    def test_recv_task_json(self):
        """Test that the TaskReceiver state can receive a task from the GCS.
        To pass the test, TaskReceiver must return the "new_task" outcome and
        the task JSON must be returned by the get() method.

        Note the  get() method must return the task JSON as a string.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        task_json = mock_types.read_mission_file("test_task.json")
        mqtt_msg = mock_types.mock_mqtt_message(
            "new_task",
            "drone/test_drone/task/new",
            task_json.encode("utf-8") 
        )
        shutdown_msg = mock_types.mock_shutdown_message()

        state.message_queue.put(mqtt_msg)
        state.message_queue.put(shutdown_msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "new_task")
        
        self.assertEqual(task_json, state.get())

    def test_recv_task_bad_json(self):
        """Test that the TaskReceiver state doesn't raise an exception when it receives
        bad JSON data. To pass the test, TaskReceiver must process the message with
        bad json data and then process the shutdown message.

        The state must return the "error" outcome. But this is fine because the
        shutdown message is triggering this outcome. The state should not raise
        an exception when it receives bad JSON data.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        # notice the missing ":" in the JSON string:
        task_json = """{
            "task": "bad_json",
            "abc" 123
        }
        """
        mqtt_msg = mock_types.mock_mqtt_message(
            "new_task",
            "drone/test_drone/task/new",
            task_json.encode("utf-8") 
        )
        shutdown_msg = mock_types.mock_shutdown_message()

        state.message_queue.put(mqtt_msg)
        state.message_queue.put(shutdown_msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "error")

    def test_recv_task_bad_json_then_good_json(self):
        """Test that the TaskReceiver state can receive a malformed task
        followed by a well-formed task.

        We want to make sure that the state can handle some bad JSON.
        We want the state to ignore the bad JSON and continue waiting for a
        well-formed task. To pass the test, the state must end in the "new_task"
        outcome and the "good" JSON must be returned by the get() method.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        # notice the missing ":" in the JSON string:
        bad_json = """{
            "task": "bad_json",
            "abc" 123
        }
        """
        good_json = mock_types.read_mission_file("test_task.json")
        all_messages = [
            mock_types.mock_mqtt_message(
                "new_task",
                "drone/test_drone/task/new",
                bad_json.encode("utf-8") 
            ),
            mock_types.mock_mqtt_message(
                "new_task",
                "drone/test_drone/task/new",
                good_json.encode("utf-8") 
            ),
            mock_types.mock_shutdown_message()
        ]

        for msg in all_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "new_task")
        self.assertEqual(good_json, state.get())
        
    def test_recv_end_task_loop(self):
        """Test that the TaskReceiver state can receive an end-task-loop message.

        This is the message that tells the drone when it's time to stop executing
        tasks. To pass the test, TaskReceiver must return the "end_task_loop" outcome.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        payload = "".encode("utf-8")
        good_json = mock_types.read_mission_file("test_task.json")
        all_messages = [
            mock_types.mock_mqtt_message(
                "end_task_loop",
                "drone/test_drone/task/end-task-loop",
                payload
            ),
            mock_types.mock_shutdown_message()
        ]

        for msg in all_messages:
            state.message_queue.put(msg)

        outcome = state.execute(None)
        self.assertEqual(outcome, "end_task_loop")

    def test_that_it_sends_the_ready_message(self):
        """Test that the TaskReceiver state sends a ready message to the GCS.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        add_position_data_to_drone(kwargs)
        mqtt_client = kwargs["mqtt_client"]

        expected_timestamp = '2024-02-08T02:00:00.001+00:00'
        mqtt_client.timestamp.return_value = expected_timestamp


        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        state.message_queue.put(mock_types.mock_message(READY_TIMER_NAME, 1.0))
        state.message_queue.put(mock_types.mock_shutdown_message())
    
        outcome = state.execute(None)
        self.assertEqual(outcome, "error")

        expected_uavid = kwargs["drone"].uav_name
        expected_topic = f"drone/{expected_uavid}/task/ready"

        expected_msg = {
            "uavid": expected_uavid,
            "timestamp": expected_timestamp,
            "position": kwargs["home"]
        }
        # mqtt_client.publish.assert_called_once_with(expected_topic, expected_msg, qos=1)
        args = mqtt_client.publish.call_args_list[0][0]
        actual_kwargs = mqtt_client.publish.call_args_list[0][1]
        self.assertEqual(actual_kwargs["qos"], 1)
        self.assertEqual(args[0], expected_topic)

        actual_msg = args[1]
        # to test the actual message, we want to make sure all the keys in the
        # expected message are in the actual message but we want to verify the
        # value of the position key separately
        for key in expected_msg:
            self.assertIn(key, actual_msg)
            if key == "position":
                continue
            self.assertEqual(actual_msg[key], expected_msg[key])

        actual_position = actual_msg["position"]
        # make sure actual position isn't none and that it's a dictionary
        self.assertIsNotNone(actual_position)
        self.assertIsInstance(actual_position, dict)

        actual_position = BriarLla.from_dict(actual_position)
        expected_position = BriarLla.from_dict(expected_msg["position"])

        # make sure the position in the ready message is almost equal to the
        # expected position. We can do this by making sure the distance between
        # the two positions is very small.
        #
        # The reason we need to do this is because we convert the LLA data to a
        # pvector before we store it in the drone.position_data object. So we
        # cannot compare the two Lla objects directly.
        expected_lla = expected_position.ellipsoid.lla
        actual_lla = actual_position.ellipsoid.lla
        
        dist = expected_lla.distance(actual_lla)
        self.assertLess(dist, 1e-7)

        