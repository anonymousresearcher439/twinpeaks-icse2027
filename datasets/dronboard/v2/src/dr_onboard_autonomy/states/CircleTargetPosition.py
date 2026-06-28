from typing import Optional

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, find_circle_start
from dr_onboard_autonomy.states import BaseState, BriarCircle, BriarWaypoint

from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


"""Example JSON:

{
    "name": "CircleTargetPosition",
    "class": "CircleTargetPosition",
    "args": {
        "target_circle_radius": 15.0,
        "target_circle_height": 15.0,
        "target_approach_speed": 4.0,
        "circle_speed": 4.0,
        "center_position":{
            "latitude": 41.60675408809003, 
            "longitude": -86.356016043474,
            "relative_altitude": 20
        },
        "sweep_angle": 360,
    },
    "transitions": [
        {
            "target": "BriarHover",
            "condition": "succeeded_circle"
        }
    ]
}

"""


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class CircleTargetPosition(BaseState):
    def __init__(self,
        target_circle_radius: float = 15.0,
        target_circle_height: float = 15.0,
        target_approach_speed: float = 4.0,
        circle_speed: float = 4.0,
        center_position: Optional[LlaDict] = None,
        sweep_angle: float = 360,
        **kwargs
    ):
        """Fly in a circle around a target position.
        The circle is defined by:
            - a target position: this is the center of the circle and where the camera will point (more on this later)
            - target_circle_radius: the radius of the circle
            - target_circle_height: the height of the circle above the target position
        These values are used to calculate a circle in the sky. The drone will follow this circle while pointing the camera at the target position.

        There are two ways this state can get it's center position (called the target position):
            1. The `center_position` argument: This is a dictionary with the `latitude`, `longitude`, and `altitude` of the target position where the altitude is meters above sea level. If this is provided, the drone will use this as the target position.
        
            2. A `target_position` message: If the `center_position` argument is not provided, this state expects the drone to have received a `target_position` message. A previous state should have processed the `target_position` message using the `TargetPosition` component. This component stores the position in the drone object. This state will read this target position and use it as the center of the circle.
            
        When the drone enters this state, it will:
            1. Fly to the edge of the circle. The drone will fly to the nearest point on the circle to its current position. It will fly at the `target_approach_speed`.

            2. The drone will then follow the circle at the `circle_speed`, while pointing the camera at the target position. You control how much of the circle the drone will fly by setting the `sweep_angle` parameter. `sweep_angle` is the number of degrees the drone will sweep while flying the circle. The sweep_angle is 360 by default, which means the drone will fly the entire circle in the clockwise direction (as seen from above). If you set the `sweep_angle` to -180, the drone will fly half the circle in the counter-clockwise direction (as seen from above).

            3. When the drone completes the circle, it will transition to the next state with the "succeeded_circle" outcome.

        Args:
            target_circle_radius: The radius of the circle in meters.
                Default is 15.0 meters

            target_circle_height: The height of the circle above the target
                position in meters. Default is 15.0 meters.
            
            target_approach_speed: The speed at which the drone will approach
                the circle in meters per second. Default is 4.0 meters per second.
            
            circle_speed: The speed at which the drone will fly in a circle in
                meters per second. Default is 4.0 meters per second.
            
            center_position: The center of the circle. This argument is optional.
                This is a dictionary with `latitude`, `longitude`, and `altitude`
                keys. All values are expected to be floats. The `altitude` value
                is in meters above sea level. If this argument is not provided,
                then the drone will read the `target_position` message to
                determine the center of the circle. 

            sweep_angle: The number of degrees the drone will sweep while flying
                the circle. The sweep_angle is 360 by default, which means the
                drone will fly the entire circle in the clockwise direction (as
                seen from above). If you set the `sweep_angle` to a negative
                number, then the drone will fly in the counter-clockwise
                direction (as seen from above).

            **kwargs: Additional arguments to pass to the underlying BriarCircle and BriarWaypoint states
        """
        required_outcomes = [
            "succeeded_circle",
            "error",
            "human_control",
            "abort",
            "rtl"
        ]
        all_outcomes = set(required_outcomes)
        if "outcomes" in kwargs:
            existing_outcomes = kwargs["outcomes"]
            for outcome in existing_outcomes:
                all_outcomes.add(outcome)
        kwargs["outcomes"] = list(all_outcomes)
        super().__init__(**kwargs)
        self._target_circle_radius = target_circle_radius
        self._target_circle_height = target_circle_height
        self._target_approach_speed = target_approach_speed
        self._circle_speed = circle_speed

        self._center_position = None
        if center_position:
            self._center_position = BriarLla(center_position, is_amsl=True)

        self._pass_through_kwargs = kwargs

        # The sweep_angle is listed as a keyword argument for documentation purposes
        # this implementation expects the sweep_angle to be a part of the _pass_through_kwargs.
        # This value is passed through to a BriarCircle state that this state creates.
        self._pass_through_kwargs["sweep_angle"] = sweep_angle


    def on_entry(self, userdata):
        if self._center_position is None:
            target_position = self.drone.data.target_position
        else:
            target_position = self._center_position
        current_pos = self.drone.data.location.get_position()
        current_pos.to_wgs84_ellipsoid()
        target_position.to_wgs84_ellipsoid()

        circle_start_position: BriarLla = find_circle_start(
            Lla(current_pos.latitude, current_pos.longitude, current_pos.altitude),
            Lla(target_position.latitude, target_position.longitude, target_position.altitude),
            self._target_circle_radius,
            self._target_circle_height
        )

        updated_current_position = self.drone.data.location.get_position()
        updated_current_position.to_wgs84_ellipsoid()

        fly_to_altitude_waypoint = BriarLla.from_args(
            updated_current_position.latitude,
            updated_current_position.longitude,
            circle_start_position.ellipsoid.lla.alt,
            is_amsl=False
        )

        target_position.to_amsl()
        fly_to_alt_state = BriarWaypoint(
            waypoint=fly_to_altitude_waypoint.amsl.dict,
            stare_position=LlaDict(
                latitude=target_position.latitude,
                longitude=target_position.longitude,
                altitude=target_position.altitude
            ),
            speed=self._target_approach_speed,
            **self._pass_through_kwargs,
        )
        fly_to_altitude_waypoint_outcome = fly_to_alt_state.execute(userdata)
        if fly_to_altitude_waypoint_outcome !=  "succeeded_waypoints":
            return fly_to_altitude_waypoint_outcome

        circle_start_waypoint = BriarWaypoint(
            waypoint=circle_start_position.amsl.dict,
            stare_position=LlaDict(
                latitude=target_position.latitude,
                longitude=target_position.longitude,
                altitude=target_position.altitude
            ),
            speed=self._target_approach_speed,
            **self._pass_through_kwargs
        )

        circle_start_outcome = circle_start_waypoint.execute(userdata)
        if circle_start_outcome != "succeeded_waypoints":
            return circle_start_outcome

        circle_target = BriarCircle(
            center_position=LlaDict(
                latitude=target_position.latitude,
                longitude=target_position.longitude,
                altitude=target_position.altitude
            ),
            stare_position=LlaDict(
                latitude=target_position.latitude,
                longitude=target_position.longitude,
                altitude=target_position.altitude
            ),
            speed=self._circle_speed,
            **self._pass_through_kwargs
        )

        circle_target_outcome = circle_target.execute(userdata)
        if circle_target_outcome != "succeeded_cirlce":
            return circle_target_outcome

        return "succeeded_circle"
