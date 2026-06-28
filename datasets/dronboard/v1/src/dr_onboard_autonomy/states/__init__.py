

from .AirLeaser import AirLeaser
from .Arm import Arm, ArmForce
from .BaseState import BaseState
from .BetterCircle import BetterCircle
from .BetterHover import BetterHover
from .BetterPath import BetterPath
from .BriarCircle import BriarCircle
from .BriarHover import BriarHover
from .BriarTravel import BriarTravel
from .BriarWaypoint import BriarWaypoint
from .CircleVisionTarget import CircleVisionTarget
from .Disarm import Disarm
from .Dropping import Dropping
from .FlyWaypoints import FlyWaypoints
from .Gimbal import GimbalTestStarePoint, GimbalTestFixedQuaternion, GimbalTestFixedEuler
from .HeartbeatHover import HeartbeatHover
from .Hover import Hover, AbortHover
from .HumanControl import HumanControl
'''
import ReadPosition needs to occer before the import of anything using ReadPosition
or an import error occurs
'''
from .ReadDroneSensors import ReadPosition
from .Land import Land
from .OnGround import OnGround
from .Offboard import Offboard
from .PhasedCircle import PhasedCircle
from .PositionDrone import PositionDrone
from .PossibleVictimDetected import PossibleVictimDetected
from .Preflight import Preflight
from .ReceiveMission import ReceiveMission
from .ReturnHome import ReturnHome
from .Rtl import Rtl
from .Searching import Searching
from .Standby import Standby
from .UnstableTakeoff import UnstableTakeoff
from .Takeoff import Takeoff
from .Tracking import Tracking
from .VictimFound import VictimFound
from .Follow_with_cvTracking import Follow_with_cvTracking

_default_transitions = {
    "AbortHover": {
        "error": "failure",
        "human_control": "HumanControl"
    },
    "Arm": {
        "error": "failure",
    },
    "ArmForce": {
        "error": "failure",
    },
    "BetterCircle": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "BetterHover": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "BetterPath": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "BriarHover": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl" 
    },
    "BriarTravel": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"     
    },
    "BriarWaypoint": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "BriarWaypoint2": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"  
    },
    "BriarWaypoint3": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"  
    },
    "BriarCircle": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "CircleVisionTarget": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "Disarm": {
        "error": "failure",
    },
    "FlyWaypoints": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "GimbalTestStarePoint": {
        "error": "failure",
    },
    "GimbalTestFixedQuaternion": {
        "error": "failure",
    },
    "GimbalTestFixedEuler": {
        "error": "failure",
    },
    "HeartbeatHover": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "Hover": {
        "error": "failure",
        "human_control": "HumanControl",
        "rtl": "Rtl"
    },
    "Land": {
        "error": "failure",
        "human_control": "HumanControl",
        "rtl": "Rtl"
    },
    "Preflight": {
        "error": "failure",
    },
    "UnstableTakeoff": {
        "error": "failure",
        "failed_takeoff": "Land",
        "human_control": "HumanControl",
    },
    "Takeoff": {
        "error": "failure",
        "failed_takeoff": "Land",
        "human_control": "HumanControl",
        "rtl": "Rtl"
    },
    "HumanControl": {
        "succeeded": "failure",
        "error": "failure",
    },
    "Dropping": {
        "error": "failure",
    },
    "OnGround": {
        "error": "failure",
    },
    "PhasedCircle": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
        "rtl": "Rtl"
    },
    "PositionDrone": {
        "error": "failure",
    },
    "PossibleVictimDetected": {
        "error": "failure",
    },
    "ReturnHome": {
        "error": "failure",
    },
    "Rtl": {
        "error": "failure",
        "human_control": "HumanControl"
    },
    "Searching": {
        "error": "failure",
    },
    "Standby": {
        "error": "failure",
    },
    "Tracking": {
        "error": "failure",
    },
    "VictimFound": {
        "error": "failure",
    },
    "Follow_with_cvTracking":{
        "abort": "AbortHover",
        "done_following": "Hover",
        "error": "failure",
        "human_control": "HumanControl",
        "rtl": "Rtl"
    }
}


def default_transitions(state):
    return _default_transitions[state].copy()

