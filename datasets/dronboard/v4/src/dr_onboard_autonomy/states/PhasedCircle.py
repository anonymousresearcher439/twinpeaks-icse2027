import copy

import rospy

from dr_onboard_autonomy.briar_helpers import (
    convert_tuple_to_LlaDict,
    LlaDict
)
from dr_onboard_autonomy.states.BaseState import BaseState
from dr_onboard_autonomy.states.BetterHover import BetterHover
from dr_onboard_autonomy.states.BriarCircle import BriarCircle
from dr_onboard_autonomy.states.BriarHover import BriarHover
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING

from typing import List


"""Example JSON:

{
    "name": "PhasedCircle",
    "class": "PhasedCircle",
    "args": {
        "pitch": 45.0,
        "distance": 30.0,
        "starting_angle": 90,
        "center_position": {
            "latitude": 41.60667484122405,
            "longitude": -86.35556383837958, 
            "relative_altitude": 0.0
        },
        "stare_position": {
            "latitude": 41.60667484122405,
            "longitude": -86.35556383837958, 
            "relative_altitude": 0.0
        },
        "total_sweep_angle": 90.0,
        "speed": 5.0,
        "cruising_altitude": 20,
        "number_arcs": 3,
        "pause_time": 4.0


    },
    "transitions": [
        {
            "target": "FlyHome",
            "condition": "succeeded_circle"
        }
    ]
}

"""


def _calc_sweep_angle(total_sweep: int, number_arcs: int) -> float:
    """
    Calculates the sweep angle for each arc
    """
    '''
    python implicitly returns a float when dividing integers
    '''
    return total_sweep / number_arcs


def _run_one_arc(
    phased_circle: 'PhasedCircle',
    circle_object: BriarCircle, 
    pause_time: float, 
    data,
    userdata,
    **kwargs):
    """
    Runs one step in sequence of an arc plus hover events
    """
    hover_args = copy.copy(kwargs)
    hover_args.update({
        'hover_time' : pause_time,
        'stare_position' : convert_tuple_to_LlaDict(circle_object.stare_position.amsl.tup),
        'data': data
    })

    briar_hover = BriarHover(**hover_args)

    rospy.loginfo("PhasedCircle: Flying next arc")
    circle_outcome = phased_circle.execute_substate(circle_object, userdata)
    
    if circle_outcome != "succeeded_circle":
        return circle_outcome

    rospy.loginfo(f"PhasedCircle: Hovering for {pause_time} seconds")
    hover_outcome = phased_circle.execute_substate(briar_hover, userdata)

    if hover_outcome != "succeeded_hover":
        return hover_outcome

    return "succeeded_circle"


def _create_arcs(
    number_arcs: int,
    center_position: LlaDict,
    stare_position: LlaDict,
    single_arc_angle: float,
    speed: float,
    data,
    **kwargs
) -> List[BriarCircle]:
    """
    Creates a list of circle instances to be run as arcs
    """
    circle_args = copy.copy(kwargs)
    circle_args.update({
        'center_position' : center_position,
        'stare_position' : stare_position,
        'sweep_angle' : single_arc_angle,
        'speed' : speed,
        'data': data
    })

    circle_list = []
    for _ in range(number_arcs):
        circle_list.append(BriarCircle(**circle_args))

    return circle_list


def _assume_start_position(
    phased_circle: 'PhasedCircle',
    pitch: float, 
    distance: float, 
    starting_angle: float, 
    center_position: LlaDict,
    pause_time: float,
    speed: float,
    cruising_altitude: float,
    data,
    userdata,
    **kwargs) -> str:
    """
    Move to starting position and orientation
    """
    '''
    update existing kwargs with inputs specifc to phased circle start position
    '''
    hover_args = copy.copy(kwargs)
    hover_args.update({
        'stare_position' : center_position,
        'pitch' : pitch,
        'distance' : distance,
        'starting_angle' : starting_angle,
        'hover_time': pause_time,
        'speed': speed,
        'cruising_altitude': cruising_altitude,
        'data': data
    })
    better_hover = BetterHover(**hover_args)
    rospy.loginfo("PhasedCircle: Flying to start position")
    return phased_circle.execute_substate(better_hover, userdata)


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class PhasedCircle(BaseState):
    def __init__(
        self,
        pitch: float,
        distance: float,
        starting_angle: float,
        center_position: LlaDict,
        stare_position: LlaDict,
        total_sweep_angle: float = 360,
        speed: float = 2.0,
        cruising_altitude = 29.0,
        number_arcs: int = 1,
        pause_time: float = 1.0,
        **kwargs
    ):
        """
        Fly a series of evenly divided arcs across a provided sweep angle and pause for a given period
        The drone's starting location is calculated using the pitch, distance, and starting_angle.
        The radius of the circle is calculated from the starting location and the center point

        Args:
            pitch: 
                (degrees) angle between center position and starting position
                It's the angle that starts between the horizontal plane and the
                ray that points from to center_position the drone.
            distance: 
                (meters) distance between center position and starting position
            starting_angle: 
                (degrees) this angle equals the compass heading from the
                center_position to the starting position. A value of 0 means the
                starting position is to the North.
            center_position: 
                has its altitude as AMSL. center of circle
            stare_position: 
                has its alt as AMSL. where the camera looks
            total_sweep_angle: 
                (degrees) total angle drone will fly after all arcs complete
            speed: 
                (m/s) speed drone will fly as moving along arcs
            cruising_altitude:
                The altitude that the drone flies on it's way to the circle.
                Specified as meters above ground level, where ground level is
                determined by the location where the drone was armed.
            number_arcs: 
                total sweep angle will be broken into this many arcs
            pause_time: 
                (s) time to pausce between flying each arc
        """
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_circle"], STANDARD_TRANSITIONS_FLYING.keys())
        
        super().__init__(**kwargs)

        self.pitch = pitch
        self.distance = distance
        self.starting_angle = starting_angle
        self.center_position = center_position
        self.stare_position = stare_position
        self.speed = speed
        self.number_arcs = number_arcs
        self.single_arc_angle = _calc_sweep_angle(total_sweep_angle, number_arcs)
        self.pause_time = pause_time
        self.cruising_altitude = cruising_altitude
        if 'name' not in kwargs:
            kwargs['name'] = self.name
        self.pass_though_kwargs = kwargs

    
    def on_entry(self, userdata):
        start_outcome = _assume_start_position(
            phased_circle=self,
            pitch=self.pitch,
            distance=self.distance,
            starting_angle=self.starting_angle,
            center_position=self.center_position,
            pause_time=self.pause_time,
            userdata=userdata,
            speed=self.speed,
            cruising_altitude=self.cruising_altitude,
            **self.pass_though_kwargs
        )
        
        if start_outcome != "succeeded_hover":
            return start_outcome
        
        circle_list = _create_arcs(
            number_arcs=self.number_arcs,
            center_position=self.center_position,
            stare_position=self.stare_position,
            single_arc_angle=self.single_arc_angle,
            speed=self.speed,
            **self.pass_though_kwargs
        )

        for circle in circle_list:
            arc_outcome = _run_one_arc(
                phased_circle=self,
                circle_object=circle,
                pause_time=self.pause_time,
                userdata=userdata,
                **self.pass_though_kwargs
            )

            if arc_outcome != "succeeded_circle":
                return arc_outcome

        return arc_outcome
