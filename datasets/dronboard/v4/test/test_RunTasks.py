import importlib
import json
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
    task_receiver.get.return_value = task_json
    return task_receiver


class TestRunTasks(unittest.TestCase):
    def test_init(self):
        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        state = RunTasks(**kwargs)

        self.assertEqual(state.mission_builder, mission_builder)
        self.assertIsInstance(state.trajectory, HoldingTrajectory)
        self.assertIsInstance(state, RunTasks)
    
    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_given_0_tasks(self, mock_TaskReceiver):
        mock_TaskReceiver.return_value = mock_types.mock_state("end_task_loop", TaskReceiver)

        mission_builder = mock_types.mock_mission_builder()
        kwargs = mission_builder.build_kwargs()
        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")

    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_given_some_tasks(self, mock_TaskReceiver):
        NUM_TASKS = 3
        task_receivers = []
        for i in range(NUM_TASKS):
            task_receiver = mock_task_receiver("new_task", {"task_id": i})
            task_receivers.append(task_receiver)
        
        task_receivers.append(mock_types.mock_state("end_task_loop", TaskReceiver))
        
        mock_TaskReceiver.side_effect = task_receivers
 

        # this is a real MissionBuilder that was initialized with lots of mock objects
        mission_builder = mock_types.mock_mission_builder()
        
        # I want to wrap the mission_builder with a mock object so that I replace the build_task method
        # I want the build_task method to return a mock task_machine
        # that always end in success
        mock_mission_builder = NonCallableMock(wraps=mission_builder)

        # lets make the task machine
        task_machine = NonCallableMock()
        task_machine.execute.return_value = "success"

        # lets make mock_mission_builder.build_task() return the task_machine
        mock_mission_builder.build_task.return_value = task_machine, []

        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder
        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")
    
    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_a_task_is_canceled(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "end_task_loop",
        ]

        task_machine_outcomes = [
            "task_canceled",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        mock_outcomes = []
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            mock_outcomes.append([])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, mock_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")

    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_some_tasks_are_canceled(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "new_task",
            "new_task",
            "new_task",
            "end_task_loop",
        ]

        task_machine_outcomes = [
            "success",
            "task_canceled",
            "task_canceled",
            "success",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        mock_extra_outcomes = []
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            mock_extra_outcomes.append([])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, mock_extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")


    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_some_tasks_have_custom_outcomes_and_some_are_canceled(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "new_task",
            "new_task",
            "new_task",
            "new_task",
            "new_task",
            "end_task_loop",
        ]

        task_machine_outcomes = [
            "custom_outcome1",
            "custom_outcome2",
            "task_canceled",
            "task_canceled",
            "success",
            "custom_outcome_last",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        mock_extra_outcomes = [
            ["custom_outcome1", "a", "b", "c"],
            ["custom_outcome2", "d", "e", "f"],
            ["a", "b", "c"],
            ["a", "b", "c"],
            ["a", "b", "c"],
            ["custom_outcome_last", "a", "b", "c"],
            [], # should not be used
        ]
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            # make the internal state that receives the task
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, mock_extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")
        self.assertEqual(state.mqtt_client.publish.call_count, 6)
        # lets check the messages that were published
        all_calls = state.mqtt_client.publish.call_args_list
        for i, (args, _) in enumerate(all_calls):
            topic, msg = args
            self.assertEqual(topic, f"drone/{kwargs['uav_id']}/task/outcome")
            self.assertIn("uavid", msg)
            self.assertIn("task_id", msg)
            self.assertIn("outcome", msg)
            self.assertIn("is_done", msg)
            self.assertIn("timestamp", msg)
            self.assertEqual(msg["uavid"], kwargs["uav_id"])
            self.assertEqual(msg["task_id"], 1)
            self.assertEqual(msg["timestamp"], state.mqtt_client.timestamp())
            expected_outcome = task_machine_outcomes[i]
            self.assertEqual(msg["outcome"], expected_outcome)
            if expected_outcome != "task_canceled":
                self.assertEqual(msg["is_done"], True)
            else:
                self.assertEqual(msg["is_done"], False)



    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_the_task_errors(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "end_task_loop", # should not be used
        ]

        task_machine_outcomes = [
            "error",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        extra_outcomes = []
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            extra_outcomes.append([])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "error")

    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_the_task_errors_and_custom_outcomes(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "end_task_loop", # should not be used
        ]

        task_machine_outcomes = [
            "error",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        extra_outcomes = []
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            extra_outcomes.append(['a', 'b', 'c'])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "error")
        state.mqtt_client.publish.assert_called_once()
        args, _ = state.mqtt_client.publish.call_args
        topic, msg = args
        self.assertEqual(topic, f"drone/{kwargs['uav_id']}/task/outcome")
        # to check the message we will make sure it has the right keys
        # and we will check specific values
        expected_keys = ["uavid", "task_id", "outcome", "is_done", "timestamp"]
        for key in expected_keys:
            self.assertIn(key, msg)
        expected_values = {
            "uavid": kwargs["uav_id"],
            "task_id": 1,
            "outcome": "error",
            "is_done": False,
        }
        for key, value in expected_values.items():
            self.assertEqual(value, msg[key])
        


    @patch.object(RunTasks_module, "TaskReceiver")
    def test_execute_when_the_task_end_in_human_control(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "end_task_loop", # should not be used
        ]

        task_machine_outcomes = [
            "failsafe",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        mock_extra_outcomes = []
        for recver_outcome, machine_outcome in zip(task_receivers_outcomes, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": 1})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            mock_extra_outcomes.append([])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, mock_extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "failsafe")





    @patch.object(RunTasks_module, "TaskReceiver")
    def test_make_sure_task_outcomes_get_reported(self, mock_TaskReceiver):
        # so here's what I want to do:
        # the task receivers will return new_task, end_task_loop
        # the task machine will return task_canceled
        # make sure this works when the task_canceled is returned

        task_receivers_outcomes = [
            "new_task",
            "new_task",
            "end_task_loop", 
        ]

        task_ids = [
            1,
            "TASK_2",
            3
        ]

        task_machine_outcomes = [
            "success",
            "task_canceled",
            "error", # should not be used
        ]

        mock_task_receivers_instances = []
        mock_task_machine_instances = []
        mock_extra_outcomes = []
        for recver_outcome, task_id, machine_outcome in zip(task_receivers_outcomes, task_ids, task_machine_outcomes):
            task_receiver = mock_task_receiver(recver_outcome, {"task_id": task_id})
            mock_task_receivers_instances.append(task_receiver)

            # lets make a mock task_machine
            task_machine = NonCallableMock()
            task_machine.execute.return_value = machine_outcome
            mock_task_machine_instances.append(task_machine)
            mock_extra_outcomes.append([])
        
        # Now we need our mock_TaskReceiver to return the mock_task_receivers_instances
        mock_TaskReceiver.side_effect = mock_task_receivers_instances

        # we also need our mission_builder to return the mock_task_machine_instances
        mission_builder = mock_types.mock_mission_builder()
        # However, we have a real MissionBuilder that was initialized with mock objects
        # I want to wrap this mission_builder in a mock object so that the
        # `build_task()` method returns a mock task_machine
        mock_mission_builder = NonCallableMock(wraps=mission_builder)
        # now we can have our mock_mission_builder return the mock_task_machine_instances we made
        mock_mission_builder.build_task.side_effect = zip(mock_task_machine_instances, mock_extra_outcomes)

        # time to initialize the RunTasks state
        kwargs = mission_builder.build_kwargs()
        kwargs["mission_builder"] = mock_mission_builder

        # we also need to mock the mqtt_client.timestamp() method
        mqtt_client = kwargs["mqtt_client"]
        mqtt_client.timestamp.return_value = '2024-02-08T02:00:18.861+00:00'

        state = RunTasks(**kwargs)
        outcome = state.execute(None)

        self.assertEqual(outcome, "success")

        # here are the expected MQTT messages
        uav_id = kwargs['uav_id']
        msgs = [
            {
                "uavid": uav_id,
                "task_id": 1,
                "outcome": "success",
                "is_done": True,
                "timestamp": '2024-02-08T02:00:18.861+00:00',
            },
            {
                "uavid": uav_id,
                "task_id": "TASK_2",
                "outcome": "task_canceled",
                "is_done": False,
                "timestamp": '2024-02-08T02:00:18.861+00:00',
            },
        ]
        mqtt_topic = f"drone/{uav_id}/task/outcome"

        mqtt_client.publish.assert_any_call(mqtt_topic, msgs[0], qos=1)
        mqtt_client.publish.assert_called_with(mqtt_topic, msgs[1], qos=1)

        # make sure we called publish twice
        self.assertEqual(mqtt_client.publish.call_count, 2)