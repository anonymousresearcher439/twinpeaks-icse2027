import rospy

from .BaseState import BaseState
from .ReceiveMission import ReceiveMissionAirborne

from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class RunMission(BaseState):
    def __init__(self, **kwargs):
        # TODO this is a hack to avoid a circular import
        from dr_onboard_autonomy.mission_helper import MissionBuilder
        """Hover and wait for at least one of each message to arrive then make
        the messages available via return_channel
        """

        # We need to add some more outcomes to make sure we respond
        # appropriately in case the remote pilot takes action

        self.all_outcomes = self._process_outcomes(kwargs, ["succeeded"], STANDARD_TRANSITIONS_FLYING.keys(), kwargs.get("unwanted_outcomes", set()))

        kwargs.update({
            'outcomes': self.all_outcomes,
            'trajectory_class': HoldingTrajectory,
        })
        super().__init__(**kwargs)

        self.mission_builder: 'MissionBuilder' = kwargs['mission_builder']


    def on_entry(self, userdata):
        kwargs = self.mission_builder.build_kwargs({
            "outcomes": self.all_outcomes,
        })

        receiver = ReceiveMissionAirborne(**kwargs)
        outcome = self.execute_substate(receiver, userdata)
        if outcome != "succeeded":
            return outcome

        mission_json = receiver.return_channel.get()
        try:
            mission = self.mission_builder.build_mission(mission_json)
        except Exception as e:
            e.print_stack()
            rospy.logerr(f"Error trying to build mission from JSON: {str(e)}")
            return "error"
        
        outcome = self.execute_substate(mission, userdata)
        if outcome != "mission_completed":
            return "error"
        return "succeeded"
