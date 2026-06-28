class MessageHandler:
    def __init__(self):
        self.passive_handlers = {}
        self.active_handlers = {}

    def add_handler(self, message_type, callback, is_active=True):
        if is_active:
            self._add_handler(self.active_handlers, message_type, callback)
        else:
            self._add_handler(self.passive_handlers, message_type, callback)

    def notify(self, message):
        message_type = message["type"]

        if message_type in self.passive_handlers:
            passive_callbacks = self.passive_handlers[message_type]
            for func in passive_callbacks:
                func(message)
        if message_type in self.active_handlers:
            active_callbacks = self.active_handlers[message_type]
            for func in active_callbacks:
                result = func(message)
                if result is not None:
                    return result
        return None

    def _add_handler(self, lookup_table, message_type, callback):
        if message_type not in lookup_table:
            lookup_table[message_type] = []
        if callback not in lookup_table[message_type]:
            lookup_table[message_type].append(callback)


from .Arm import Arm, ArmForce
from .BaseState import BaseState
from .BetterCircle import BetterCircle
from .BetterHover import BetterHover
from .BetterPath import BetterPath
from .BriarCircle import BriarCircle
from .BriarHover import BriarHover
from .BriarTravel import BriarTravel
from .BriarWaypoint import BriarWaypoint
from .Disarm import Disarm
from .Dropping import Dropping
from .FlyWaypoints import FlyWaypoints
from .Hover import Hover, AbortHover
from .HumanControl import HumanControl
from .Land import Land
from .OnGround import OnGround
from .PhasedCircle import PhasedCircle
from .PositionDrone import PositionDrone
from .PossibleVictimDetected import PossibleVictimDetected
from .Preflight import Preflight
from .ReceiveMission import ReceiveMission
from .ReturnHome import ReturnHome
from .Searching import Searching
from .Standby import Standby
from .Takeoff import Takeoff
from .Tracking import Tracking
from .VictimFound import VictimFound
from .Follow_with_cvTracking import Follow_with_cvTracking

_default_transitions = {
    "AbortHover": {
        "error": "failure",
        "human_control": "HumanControl",
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
    },
    "BetterHover": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
    },
    "BetterPath": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
    },
    "BriarHover": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",  
    },
    "BriarTravel": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",        
    },
    "BriarWaypoint": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",        
    },
    "BriarWaypoint2": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",        
    },
    "BriarCircle": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
    },
    "Disarm": {
        "error": "failure",
    },
    "FlyWaypoints": {
        "error": "failure",
        "human_control": "HumanControl",
        "abort": "AbortHover",
    },
    "Hover": {
        "error": "failure",
        "human_control": "HumanControl",
    },
    "Land": {
        "error": "failure",
        "human_control": "HumanControl",
    },
    "Preflight": {
        "error": "failure",
    },
    "Takeoff": {
        "error": "failure",
        "failed_takeoff": "Land",
        "human_control": "HumanControl",
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

        "error": "failure",
        "done_following": "Hover",
    }
}


def default_transitions(state):
    return _default_transitions[state].copy()

