from typing import Optional

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    LlaDict
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.BriarHover import BriarHover
from dr_onboard_autonomy.states.components import SadeCommunication
from dr_onboard_autonomy.states.components.trajectory import YawTrajectory, HoldingTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from .BaseState import BaseState


"""Example JSON:

{
    "name": "SadeEnter",
    "class": "SadeEnter",
    "args": {
        "sade_zone_id": "PPT",
        "timeout": 15.0,
    },
    "transitions": [
        {
            "target": "BriarTravel",
            "condition": "allow"
        },
        {
            "target": "FlyHome",
            "condition": "deny"
        },
        {
            "target": "FlyToProvingGround",
            "condition": "proving_ground"
        },
        {
            "target": "FlyHome",
            "condition": "timeout"
        }
    ]
}

"""


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class SadeEnter(BriarHover):
    def __init__(self, sade_zone_id: str, timeout: float = 15.0, **kwargs):
        """
        Enters a SADE zone and hovers while waiting for a decision.

        Args:
            sade_zone_id: The ID of the SADE zone to enter.
            timeout: How long to wait for a decision before exiting.
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["allow", "deny", "proving_ground", "timeout"], STANDARD_TRANSITIONS_FLYING.keys())

        kwargs["unwanted_outcomes"] = ["succeeded_hover"]
        
        super().__init__(hover_time=-1, **kwargs)

        # TODO uncomment these assertions
        # assert "drone_registration_id" in kwargs
        # assert "pilot_id" in kwargs
        self.drone_registration_id = kwargs.get("drone_registration_id", "UKNOWN DRONE REGISTRATION ID")
        self.pilot_id = kwargs.get("pilot_id", "UKNOWN PILOT ID")

        self.sade_zone_id = sade_zone_id
        self.sade_communication = SadeCommunication(
            self,
            sade_zone_id,
            self.drone_registration_id,
            self.pilot_id,
            owner_id=kwargs.get("owner_id", "UNKNOWN OWNER ID"),
            model_name=kwargs.get("model_name", "UNKNOWN MODEL NAME"),
            enable_entry_outcomes=True,
            enable_exit_outcomes=False
        )

        self.message_senders.add(RepeatTimer("timeout", timeout))
        self.handlers.add_handler("timeout", self.on_timeout)

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self._send_sade_messages)
        self._is_sent = False
    
    def _send_sade_messages(self, userdata):
        if self._is_sent:
            return
        self.sade_communication.send_uav_registration()
        self.sade_communication.send_pilot_registration()
        self.sade_communication.send_entry_request()
        self._is_sent = True
    
    def on_timeout(self, userdata):
        return "timeout"


"""Example JSON:

{
    "name": "SadeExit",
    "class": "SadeExit",
    "args": {
        "sade_zone_id": "PPT",
        "timeout": 15.0,
    },
    "transitions": [
        {
            "target": "FlyHome",
            "condition": "success"
        },
        {
            "target": "FlyHome",
            "condition": "timeout"
        },
    ]
}

"""

@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class SadeExit(BriarHover):
    def __init__(self, sade_zone_id: str, timeout: float = 15.0, **kwargs):
        """
        Exits a SADE zone and hovers while waiting for a confirmation.

        Args:
            sade_zone_id: The ID of the SADE zone to exit.
            timeout: How long to wait for a confirmation before exiting.
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["success_sade_exit", "timeout"], STANDARD_TRANSITIONS_FLYING.keys())
        kwargs["unwanted_outcomes"] = ["succeeded_hover"]
        
        super().__init__(hover_time=-1, **kwargs)

        self.drone_registration_id = kwargs.get("drone_registration_id", "UKNOWN DRONE REGISTRATION ID")
        self.pilot_id = kwargs.get("pilot_id", "UKNOWN PILOT ID")
        self.sade_zone_id = sade_zone_id

        self.sade_communication = SadeCommunication(
            self,
            sade_zone_id,
            self.drone_registration_id,
            self.pilot_id,
            owner_id=kwargs.get("owner_id", "UNKNOWN OWNER ID"),
            model_name=kwargs.get("model_name", "UNKNOWN MODEL NAME"),
            enable_entry_outcomes=False,
            enable_exit_outcomes=True
        )

        self.message_senders.add(RepeatTimer("timeout", timeout))
        self.handlers.add_handler("timeout", self.on_timeout)

        # We wait to receive the state message before sending the SADE messages
        # This is to ensure that the state is ready to receive the messages
        # before we send them. (This avoid a race condition where the response
        # arrives before can receive it)
        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self._send_sade_messages)
        self._is_sent = False
    
    def _send_sade_messages(self, userdata):
        if self._is_sent:
            return
        self.sade_communication.send_sade_exit()
        self._is_sent = True

    def on_timeout(self, userdata):
        return "timeout"