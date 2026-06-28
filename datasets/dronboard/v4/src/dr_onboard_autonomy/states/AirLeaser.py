import json
from enum import Enum, auto
import random
from typing import (
    Dict,
    List,
    Optional,
    Set,
    Type,
    Union,
)
from dr_onboard_autonomy.models.kinematics import TimeStampedLlaPosition
from dr_onboard_autonomy.state_factory import STANDARD_TRANSITIONS_FLYING
import rospy

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.airlease import (
    AirLeaseCallback,
    AirLeaseOutcome,
    AirspaceVolume,
    AirspaceVolumeName,
    TunnelFunc,
)
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.BaseState import BaseState
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory

class AirLeaser(BaseState):
    """This state asks for an air lease until one is granted

    Args:
        tunnel_func Optional[TunnelFunc]: Generates air lease Request args - provided from closures in 
            dr_onboard_autonomy.air_lease
        
        is_flying bool: are we flying? This controls how AirLeaser initializes its internal state 
            if `is_flying` is True, then we will hover in place while we send requests and
            wait for permission to fly. Furthermore we will include certain outcomes by default
            such as the "failsafe" outcome.

            But if `is_flying` is False, then we will not send any setpoints to the flight controler
            and we will only include the "succeed" and "error" outcomes.

            Most of the time this should be True. Set this to false for Takeoff. 

        unwanted_outcomes Set[str]: this is a set of strings. These are outcomes to exclude from the outcome
            argument passed to smach.State. For example, the Takeoff state doesn't want the
            "failsafe" outcome to be possible before lift-off. By default this is the empty
            set. Most of the time you can use the default.
    """

    class Mode(Enum):
        STARTING = auto()
        WAITING = auto()
        DONE = auto()

    def __init__(
        self,
        tunnel_func: Optional[TunnelFunc]=None,
        is_flying: bool=True,
        unwanted_outcomes: Set[str]=set(),
        **kwargs
    ):
        if is_flying:
            # We're in FLYING_MODE
            required_outcomes = ["succeeded", *STANDARD_TRANSITIONS_FLYING.keys()]
            TrajectoryClass = HoldingTrajectory
        else:
            # We're in GROUND_MODE
            required_outcomes = [
                "error",
                "succeeded",
            ]
            TrajectoryClass = None
        
        outcome_set = set(kwargs.get("outcomes", []))
        for other_outcome in required_outcomes:
            outcome_set.add(other_outcome)
        outcome_set = outcome_set - unwanted_outcomes
        self.outcome_set = outcome_set

        kwargs["outcomes"] = list(outcome_set)
        kwargs["trajectory_class"] = TrajectoryClass
        super().__init__(**kwargs)
        
        assert tunnel_func is not None, "AirLeaser requires a tunnel_func to be provided"
        assert self.reusable_message_senders is not None, "AirLeaser requires reusable_message_senders to be provided"

        self._exit_on_deadlock = "deadlock" in kwargs.get("outcomes", [])
        self.unwanted_outcomes = unwanted_outcomes

        self.tunnel_func: TunnelFunc = tunnel_func
        self.airspace_volume: Optional[AirspaceVolume] = None
        
        self.mission_builder: 'MissionBuilder' = kwargs["mission_builder"]

        self.air_lease = AirLease(self, cleanup_messages=False)
        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position)

        self.mode = AirLeaser.Mode.STARTING
        self._outcome: Optional[str] = None
        
    
    def on_position(self, message: dict):
        if self.mode == AirLeaser.Mode.STARTING:
            self.mode = AirLeaser.Mode.WAITING
            pos: TimeStampedLlaPosition = message["data"]
            airspace = self.tunnel_func(pos)
            self.airspace_volume = airspace
            name = self.air_lease.request_airspace(airspace, self.on_air_lease_outcome)
            rospy.loginfo(f"AirLeaser - requested airspace {name} from {airspace[0].start} to {airspace[-1].end}")
        
        elif self.mode == AirLeaser.Mode.DONE:
            return self._outcome
    
    def on_air_lease_outcome(self, name: AirspaceVolumeName, airspace: AirspaceVolume, outcome: AirLeaseOutcome):
        destination = airspace[-1].end
        rospy.loginfo(f"AirLeaser - received response from air leasing service. AirspaceVolume: {name}. Outcome: {outcome}. Destination: {destination}")
        if outcome.deadlock:
            rospy.logwarn(f"AirLeaser - DEADLOCK WARNING! airspace {name} to {destination} is causing deadlocked")
            if self._exit_on_deadlock:
                self._outcome = "deadlock"
                self.mode = AirLeaser.Mode.DONE
                return
        
        if outcome.approved:
            rospy.loginfo(f"AirLeaser - airspace {name}  to {destination} was approved with Lease ID: {self.air_lease.lease_id}")
            self._outcome = "succeeded"
            self.mode = AirLeaser.Mode.DONE
        else:
            rospy.logwarn(f"AirLeaser - air space {name} to {destination} was denied: {outcome}")



from dr_onboard_autonomy.states.components import AirLease
from dr_onboard_autonomy.mission_helper import MissionBuilder
