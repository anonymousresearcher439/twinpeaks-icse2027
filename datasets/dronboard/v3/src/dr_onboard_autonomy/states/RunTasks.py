from datetime import datetime
import json
from queue import Empty
from typing import List, Optional, TypedDict, Union
import rospy

from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from .BaseState import BaseState
from .TaskReceiver import TaskReceiver




@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class RunTasks(BaseState):
    def __init__(self, **kwargs):
        self.all_outcomes = self._process_outcomes(kwargs, ["success"], STANDARD_TRANSITIONS_FLYING.keys())
        kwargs["outcomes"] = self.all_outcomes

        super().__init__(trajectory_class=HoldingTrajectory, **kwargs)

        self.MQTT_TOPIC = f"drone/{self.drone.uav_name}/task/outcome"

        self.mission_builder: 'MissionBuilder' = kwargs['mission_builder']

        # TODO this is a hack to avoid a circular import
        from dr_onboard_autonomy.mission_helper import MissionBuilder
    
    def on_entry(self, userdata):
        super().on_entry(userdata)
        previous_task_outcome = None
        while True:
            # we need to get a task
            kwargs = self.mission_builder.build_kwargs({
                "outcomes": self.all_outcomes,
                "name": self.name,
                "previous_task_outcome": previous_task_outcome,
            })
            task_receiver = TaskReceiver(**kwargs)
            outcome = self.execute_substate(task_receiver, userdata)
            if outcome == "end_task_loop":
                # We received the `end-task-loop` message from the GCS
                # This is how the `RunTasks` state determines
                # it should stop running tasks 
                return "success"
            if outcome != "new_task":
                # something atypical happened
                # maybe the pilot took control or something
                return outcome

            # if we made it this far, then we got a new task to run… 
            # 2. Build a state machine for the task
            try:
                task_json: str = task_receiver.get()
                task_machine, custom_outcomes = self.mission_builder.build_task(task_json)
            except Empty as _empty:
                rospy.logerr(f"TaskReceiver did not receive a task: {_empty.__traceback__}")
                continue
            except ValueError as _value_error:
                rospy.logerr(f"Failed to build task machine from task: {task_json}")
                rospy.logdebug(f"Error: {_value_error}")
                continue
            # note RunTasks can be small in part because `mission_builder` would
            # do some heavy lifting in its `build_mission` method.

            # 3. run the task
            outcome = self.execute_substate(task_machine, userdata)
            
            # 4. report back to ground control
            task_id = self._get_task_id(task_json)
            if task_id and self.is_task_done(outcome):
                previous_task_ids = self.data.get("task_ids", set())
                previous_task_ids.add(task_id)
                self.data["task_ids"] = previous_task_ids
            ordinary_outcomes = ["success", "task_canceled"]
            ordinary_outcomes.extend(custom_outcomes)
            msg = self._create_task_outcome_report(task_id, outcome, ordinary_outcomes)
            previous_task_outcome = msg
            if outcome not in ordinary_outcomes:
                # We stopped executing the task b/c of an atypical outcome like:
                # "low_battery", "error", "failsafe", "abort", or "rtl"
                return outcome
            # else:
            # continue the loop

    def _create_task_outcome_report(self, task_id: Optional[Union[int, str]], outcome: str, ordinary_outcomes: List[str]):
        is_done = outcome in ordinary_outcomes and outcome != "task_canceled"
        msg = {
            "uavid": self.drone.uav_name,
            "task_id": task_id,
            "timestamp": self.mqtt_client.timestamp(),
            "outcome": outcome,
            "is_done": is_done,
        }

        self.mqtt_client.publish(self.MQTT_TOPIC, msg, qos=1)
        return msg
    
    def _get_task_id(self, task_json: str):
        task_spec = json.loads(task_json)
        return task_spec.get("task_id")

    def is_task_done(self, outcome: str):
        if outcome and outcome.startswith('_'):
            return False
        if outcome in [
            'low_battery',
            'error',
            'failsafe',
            'abort',
            'rtl',
            'task_canceled'
        ]:
            return False
        return True