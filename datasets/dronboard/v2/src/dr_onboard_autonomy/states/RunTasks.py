from datetime import datetime
import json
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
        outcomes_set = {
            "success",
            "error",
            "human_control",
            "abort",
            "rtl",
        }
        if "outcomes" in kwargs:
            outcomes_set.update(kwargs["outcomes"])
        self.all_outcomes = list(outcomes_set)
        kwargs["outcomes"] = self.all_outcomes

        super().__init__(trajectory_class=HoldingTrajectory, **kwargs)

        self.MQTT_TOPIC = f"drone/{self.drone.uav_name}/task/outcome"

        self.mission_builder: 'MissionBuilder' = kwargs['mission_builder']

        # TODO this is a hack to avoid a circular import
        from dr_onboard_autonomy.mission_helper import MissionBuilder
    
    def on_entry(self, userdata):
        super().on_entry(userdata)
    
        while True:
            # we need to get a task
            kwargs = self.mission_builder.build_kwargs({
                "outcomes": self.all_outcomes,
                "name": self.name,
            })
            task_receiver = TaskReceiver(**kwargs)
            outcome = task_receiver.execute(userdata)
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
            task_json: str = task_receiver.get()
            task_machine = self.mission_builder.build_task(task_json)
            # note RunTasks can be small in part because `mission_builder` would
            # do some heavy lifting in its `build_mission` method.

            # 3. run the task
            outcome = task_machine.execute(userdata)
            if outcome not in ["success", "task_canceled"]:
                # We stopped executing the task b/c of an atypical outcome like:
                # "error", "human_control", "abort", or "rtl"
                return outcome
            
            # 4. report back to ground control
            task_id = self._get_task_id(task_json)
            self._report_task_outcome(task_id, outcome)
            # continue the loop

    def _report_task_outcome(self, task_id: Optional[Union[int, str]], outcome: str):
        is_done = "success" == outcome
        msg = {
            "uavid": self.drone.uav_name,
            "task_id": task_id,
            "timestamp": self.mqtt_client.timestamp(),
            "outcome": outcome,
            "is_done": is_done,
        }

        self.mqtt_client.publish(self.MQTT_TOPIC, msg, qos=1)
    
    def _get_task_id(self, task_json: str):
        task_spec = json.loads(task_json)
        return task_spec.get("task_id")