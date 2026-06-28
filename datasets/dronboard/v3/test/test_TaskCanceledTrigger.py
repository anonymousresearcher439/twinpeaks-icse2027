import importlib
import json
import unittest

from unittest.mock import patch, NonCallableMock, Mock

from dr_onboard_autonomy.states.components import TaskCanceledTrigger

from . import mock_types


RunTasks_module = importlib.import_module(TaskCanceledTrigger.__module__)


class TestTaskCanceledTrigger(unittest.TestCase):
    def test_is_valid_task_id(self):
        state = mock_types.mock_base_state()
        valid_ids = [
            0,
            1,
            2000,
            "SOME_TASK_ID",
            " ", # TODO Should this be valid? I think it should be invalid... but it's not an empty string...
        ]
        invalid_ids = [
            None,
            "",
            0.0,
            0.1,
            -1,
            -2000,
            True,
            False,
        ]
        for task_id in valid_ids:
            trigger = TaskCanceledTrigger(state, task_id)
            self.assertTrue(trigger.is_valid_task_id(), f"task_id: '{task_id}' is valid but is_valid_task_id() returned False")
        
        for task_id in invalid_ids:
            trigger = TaskCanceledTrigger(state, task_id)
            self.assertFalse(trigger.is_valid_task_id(), f"task_id: '{task_id}' is invalid but the is_valid_task_id() method behaves as if it's valid")
        
    
    def test_on_cancel_message(self):
        state = mock_types.mock_base_state()

        trigger_with_valid_id = TaskCanceledTrigger(state, "SOME_TASK_ID")
        trigger_with_invalid_id = TaskCanceledTrigger(state, -1)

        mqtt_topic = "drone/TEST_DRONE/cancel-current-task"

        # All of these task_ids do not match the task_id in trigger_with_valid_id
        task_ids = [
            "ANOTHER_TASK_ID",
            "YET_ANOTHER_TASK_ID",
            1,
            2,
        ]

        for task_id in task_ids:
            payload_data = {
                "task_id": task_id,
            }
            payload_str = json.dumps(payload_data)
            
            payload = payload_str.encode("utf-8")
            msg = mock_types.mock_mqtt_message("cancel_current_task", mqtt_topic, payload)
            
            outcome = trigger_with_valid_id.on_cancel_message(msg)
            # if the outcome is None then the task was not canceled
            self.assertIsNone(outcome, f"That task ID didn't match but the TaskCanceledTrigger returned '{outcome}' instead of None. The task_id in the message was '{task_id}' and component should only return an outcome when the task_id is '{trigger_with_valid_id.task_id}'")

            outcome = trigger_with_invalid_id.on_cancel_message(msg)
            # in this case the task_id is invalid so the task should be canceled always
            self.assertEqual(outcome, "task_canceled")

        # let's test the case where we have a valid task_id and the task_id matches
        payload_data = {
            "task_id": trigger_with_valid_id.task_id,
        }
        payload = json.dumps(payload_data).encode("utf-8")

        msg = mock_types.mock_mqtt_message("cancel_current_task", mqtt_topic, payload)
        outcome = trigger_with_valid_id.on_cancel_message(msg)
        self.assertEqual(outcome, "task_canceled")
        
