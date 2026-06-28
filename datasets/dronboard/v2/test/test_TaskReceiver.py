import importlib
import json
import queue
import sys
import time
import unittest

from unittest.mock import patch, NonCallableMock, Mock

from dr_onboard_autonomy.states import RunTasks, TaskReceiver
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory

from . import mock_types


RunTasks_module = importlib.import_module(RunTasks.__module__)


def mock_task_receiver(outcome, task_dict):
    task_receiver = mock_types.mock_state(outcome, TaskReceiver)
    task_json = json.dumps(task_dict)
    task_receiver.get.return_value = task_json.encode("utf-8")
    return task_receiver


class TestTaskReceiver(unittest.TestCase):
    def test_init(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
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
        """Test that the TaskReceiver state can receive a task from the GCS.
        To pass the test, TaskReceiver must return the "new_task" outcome and
        the task JSON must be returned by the get() method.

        Note the  get() method must return the task JSON as a string.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
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
        """Test that the TaskReceiver state can receive a task from the GCS.
        To pass the test, TaskReceiver must return the "new_task" outcome and
        the task JSON must be returned by the get() method.

        Note the  get() method must return the task JSON as a string.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
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
        """Test that the TaskReceiver state can receive a task from the GCS.
        To pass the test, TaskReceiver must return the "new_task" outcome and
        the task JSON must be returned by the get() method.

        Note the  get() method must return the task JSON as a string.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
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
        """Test that the TaskReceiver state can receive a task from the GCS.
        To pass the test, TaskReceiver must return the "new_task" outcome and
        the task JSON must be returned by the get() method.

        Note the  get() method must return the task JSON as a string.
        """
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        mqtt_client = kwargs["mqtt_client"]

        expected_timestamp = '2024-02-08T02:00:00.001+00:00'
        mqtt_client.timestamp.return_value = expected_timestamp


        state = TaskReceiver(**kwargs)
        state.message_queue = queue.Queue()

        state.message_queue.put(mock_types.mock_shutdown_message())
    
        outcome = state.execute(None)
        self.assertEqual(outcome, "error")

        expected_uavid = kwargs["drone"].uav_name
        expected_topic = f"drone/{expected_uavid}/task/ready"
        expected_msg = {
            "uavid": expected_uavid,
            "timestamp": expected_timestamp,
        }
        mqtt_client.publish.assert_called_once_with(expected_topic, expected_msg, qos=1)