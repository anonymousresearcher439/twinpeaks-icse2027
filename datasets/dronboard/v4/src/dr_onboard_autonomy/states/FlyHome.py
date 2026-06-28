from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict
from dr_onboard_autonomy.models import LlaPosition
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.Waypoint import Waypoint
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory


"""Example JSON:

{
    "name": "FlyHome",
    "class": "FlyHome",
    "args": {
        "speed": 5.0,
        "stare_pitch": 45.0
    },
    "transitions": [
        {
            "condition": "succeeded_waypoints",
            "target": "Land"
        }
    ]
}

"""

@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class FlyHome(BaseState):

    def __init__(self,
                 home: LlaDict,
                 speed: float = 6.0,
                 stare_position: LlaDict = None,
                 stare_pitch: float = None,
                 **kwargs):
        """
        Fly home at the current altitude taking the most direct path.

        Args:
            Home (LlaDict): The home location specified as a dictionary with the following keys:
                - latitude (float): Latitude in degrees.
                - longitude (float): Longitude in degrees.
                - altitude (float): Altitude in meters AMSL.
            
            speed (float, optional): Speed to fly in meters per second. Defaults to 6.0 m/s.

            stare_position (LlaDict, optional): A dictionary specifying where the camera should look. It has the following keys:
                - latitude (float): Latitude in degrees.
                - longitude (float): Longitude in degrees.
                - altitude (float): Altitude in meters AMSL.

            stare_pitch (float, optional): Controls how far down to pitch the camera in degrees. Specifically, it represents the rotation in degrees around the camera's LEFT axis. For example, a positive number like 45° makes the camera look down by 45°. If both `stare_pitch` and `stare_position` are set, `stare_position` takes priority.
        """        
        unwanted_outcomes = kwargs.get("unwanted_outcomes", set())
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_waypoints"], STANDARD_TRANSITIONS_FLYING.keys(), unwanted_outcomes)
        kwargs.update({
            "home": home,
            "trajectory_class": WaypointTrajectory,
        })

        super().__init__(**kwargs)

        self.home = BriarLla.from_dict(home, is_amsl=True)
        self.speed = speed
        self.mission_builder: 'MissionBuilder' = kwargs["mission_builder"]

        self.stare_position = stare_position
        if self.stare_position is not None:
            self.stare_position = BriarLla.from_dict(self.stare_position, is_amsl=True)
        self.stare_pitch = stare_pitch
        self.unwanted_outcomes = unwanted_outcomes

    def on_entry(self, userdata):
        # Read the current position so we can find our altitude
        pos_reader_args = self.mission_builder.build_kwargs({
            "message_names": ["position"],
            "name": self.name,
            "unwanted_outcomes": self.unwanted_outcomes,
        })
        position_reader = ReadMessagesAirborne(**pos_reader_args)
        outcome = self.execute_substate(position_reader, userdata)
        if outcome != "succeeded":
            return outcome
        output = position_reader.get()
        lla_pos: LlaPosition = output["position"]
        pos = BriarLla(LlaDict(
                latitude=lla_pos.latitude,
                longitude=lla_pos.longitude,
                altitude=lla_pos.altitude
            ),
            is_amsl=False
        )

        # calculate our waypoint so that it's above the home location at the current altitude
        altitude = pos.amsl.lla.altitude
        home_lla = self.home.amsl.lla
        waypoint = BriarLla.from_args(home_lla.latitude, home_lla.longitude, altitude, is_amsl=True)

        # fly to the waypoint
        waypoint_args = self.mission_builder.build_kwargs({
            "waypoint":waypoint.amsl.dict,
            "speed": self.speed,
            "name": self.name,
            "unwanted_outcomes": self.unwanted_outcomes,
        })

        if self.stare_position is not None:
            waypoint_args['stare_position'] = self.stare_position.amsl.dict

        if self.stare_pitch is not None:
            waypoint_args['stare_pitch'] = self.stare_pitch

        waypoint_state = Waypoint(**waypoint_args)
        return self.execute_substate(waypoint_state, userdata)


@register_state(default_transitions={
    "error": "failure",
    "failsafe": "Failsafe",
    "abort": "AbortHover",
    "rtl": "Rtl",
})
class ReturnToRecharge(FlyHome):
    def __init__(self, **kwargs):
        kwargs["unwanted_outcomes"] = {'low_battery', 'succeeded_waypoints'}
        kwargs['outcomes'] = ['mission_completed', 'error', 'failsafe', 'abort', 'rtl']
        super().__init__(**kwargs)
    
    def on_entry(self, userdata):
        outcome = super().on_entry(userdata)
        if outcome != "succeeded_waypoints":
            return outcome
        landing_args = self.mission_builder.build_kwargs({
            "name": self.name,
        })
        lander = Land(**landing_args)
        outcome = self.execute_substate(lander, userdata)
        if outcome != "succeeded_land":
            return outcome
        
        disarm_args = self.mission_builder.build_kwargs({
            "name": self.name,
        })
        disarm = Disarm(**disarm_args)
        outcome = self.execute_substate(disarm, userdata)
        if outcome != "succeeded_disarm":
            return outcome
        return "mission_completed"



from dr_onboard_autonomy.mission_helper import MissionBuilder
from dr_onboard_autonomy.states import BriarWaypoint, ReadMessagesAirborne, Disarm, Land
