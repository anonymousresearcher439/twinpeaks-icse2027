from droneresponse_mathtools import Lla, Pvector
import json
from math import floor
from queue import Queue
from typing import Tuple, NamedTuple

import rospy

from dr_onboard_autonomy.gimbal.data_types import Quaternion

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    briarlla_from_lat_lon_alt,
    convert_Lla_to_LlaDict,
)
from dr_onboard_autonomy.gimbal.geolocation import geolocate_object_from_camera
from dr_onboard_autonomy.gimbal.gimbal_math2 import GimbalCalculator2
from dr_onboard_autonomy.states import BaseState, BriarHover, BriarWaypoint
from dr_onboard_autonomy.states.BriarCircle import BriarCircle
from dr_onboard_autonomy.states.ReadDroneSensors import ReadMessages

CalculationData = NamedTuple("CalculationData", [
    ("drone_position", Tuple[float, float, float]),
    ("drone_attitude", Tuple[float, float, float, float]),
    ("gimbal_attitude", Tuple[float, float, float, float]),
])



def _get_circle_start_lat_lon(
    target_pos: Lla,
    target_radius: float,
    target_circle_height: float
) -> Lla:
    '''
    find circle start location east of target by radius and up from target by target circle height
    '''
    return(target_pos.move_ned(0.0, target_radius, -target_circle_height))
    '''
    Could get clever and move to start position on circle closest to to the drone's current position
    - tougher because need to get the current drone position into the target's local coordinate
    system
    - next figure out the angle in that coordinate system (in NED) between the two
    - then can move to the buffer radius and height
    '''


def _image_to_pixel_csys(image_x: float, image_y: float, x_res: float, y_res: float
) -> Tuple[int, int]:
    """Converts image plane coordinates to pixel coordinates

    Args:
        image_x:
            x goes from 0 to 1 where 0 is the left and 1 is the right of the image
        image_y:
            y goes from 0 to 1 where 0 is the top and 1 is the bottom of the image
        x_res:
            number of pixels in horizonal direction
        y_res:
            number of pixels in vertical direction
    """
    return (floor(image_x * x_res), floor(image_y * y_res))


class CircleVisionTarget(BaseState):
    """Circles target found by vision service

    Args:
        target_circle_radius: radius of circle to be flown around the target
        target_circle_height: height above the target the circle will be flown (meters)
        target_approach_speed: flight speed to the circle start location
        circle_speed: flight speed for circle around target (meters / second)

    Assumes the vision service continues to send messages of a target's location after a target is
    found
    """
    def __init__(
        self,
        target_circle_radius: float=15.0,
        target_circle_height: float=15.0,
        target_approach_speed: float=4.0,
        circle_speed: float=4.0,
        data: dict=None,
        **kwargs
    ):
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
        self._data=data
        self._messages_return_channel = Queue()
        self._read_messages = ReadMessages(
            message_names=["vision"],
            return_channel=self._messages_return_channel,
            **kwargs
        )
        self._read_drone_data = ReadMessages(
            message_names=["position", "imu"],
            return_channel=self._messages_return_channel,
            **kwargs
        )
        self._pass_through_kwargs = kwargs


    def on_entry(self, userdata):
        briar_hover = BriarHover(hover_time=0.1, **self._pass_through_kwargs)
        hover_outcome = briar_hover.execute(userdata)
        if hover_outcome != "succeeded_hover":
            return hover_outcome

        outcome = self._read_messages.execute(userdata)
        if outcome != "succeeded":
            return outcome

        vision_data: dict = json.loads(self._messages_return_channel.get()["vision"].payload)
        ts = vision_data['ts']

        # if ts is newer than drone pos or attitude
        t_pos_newest, _ = self.drone.position_data.last()
        t_att_newest, _ = self.drone.attitude_data.last()
        if ts > t_pos_newest or ts > t_att_newest:
            outcome = self._read_drone_data.execute(userdata)
            if outcome != "succeeded":
                return outcome
        
        try:
            drone_pos, drone_quaternion, gimbal_quaternion = self.find_frames(ts)

        except:
            return "error"

        current_pos = briarlla_from_lat_lon_alt(*Pvector(*drone_pos).to_lla(), is_amsl=False)

        '''
        fov_h used below is specific to the IMX477
        ideally would be populated by a config specific to the UAV
        '''
        gimbal_attitude_world = GimbalCalculator2.attitude_drone_to_world_frame(
            gimbal_attitude=Quaternion(
                gimbal_quaternion[0],
                gimbal_quaternion[1],
                gimbal_quaternion[2],
                gimbal_quaternion[3],
            ),
            drone_attitude=Quaternion(
                drone_quaternion[0],
                drone_quaternion[1],
                drone_quaternion[2],
                drone_quaternion[3],
            )
        )

        target_location: BriarLla = briarlla_from_lat_lon_alt(
            *(
                *geolocate_object_from_camera(
                    fov_h=80,
                    lla=current_pos.ellipsoid.tup,
                    image_res=(vision_data["x_res"], vision_data["y_res"]),
                    target_coords=(vision_data["x"], vision_data["y"]),
                    quaternion_gimbal=gimbal_attitude_world.astuple(),
                    ground_alt=self._data["arm_position"].altitude
                ),
                False
            )
        )

        circle_start_lla = _get_circle_start_lat_lon(
            target_pos=target_location.ellipsoid.lla,
            target_radius=self._target_circle_radius,
            target_circle_height=self._target_circle_height
        )

        circle_start = BriarLla(
            pos=convert_Lla_to_LlaDict(
                circle_start_lla
            ),
            is_amsl=False
        )

        circle_start_waypoint = BriarWaypoint(
            waypoint=circle_start.amsl.dict,
            stare_position=target_location.amsl.dict, 
            speed=self._target_approach_speed,
            **self._pass_through_kwargs
        )
        circle_start_outcome = circle_start_waypoint.execute(userdata)
        if circle_start_outcome != "succeeded_waypoints":
            return circle_start_outcome

        circle = BriarCircle(
            center_position=target_location.amsl.dict,
            stare_position=target_location.amsl.dict,
            speed=self._circle_speed,
            **self._pass_through_kwargs
        )
        circle_outcome = circle.execute(userdata)
        if circle_outcome != "succeeded_circle":
            return circle_outcome

        return "succeeded_circle"

    def find_frames(self, ts: float) -> CalculationData:
        t_gimbal_newest, _ = self.drone.gimbal.attitude_data.last()        
        try:
            drone_pos = self.drone.position_data.lookup(ts)
            drone_quaternion = self.drone.attitude_data.lookup(ts)
            if ts > t_gimbal_newest:
                gimbal_quaternion = self.drone.gimbal.attitude_data.last()
            else:
                gimbal_quaternion = self.drone.gimbal.attitude_data.lookup(ts)

            return CalculationData(drone_attitude=drone_quaternion, drone_position=drone_pos, gimbal_attitude=gimbal_quaternion)
            
        except Exception as e:
            t_att_oldest, _ = self.drone.attitude_data[0]
            t_pos_oldest, _ = self.drone.position_data[0]
            t_gimbal_oldest, _ = self.drone.gimbal.attitude_data[0]
            if ts <= max(t_att_oldest, t_pos_oldest, t_gimbal_oldest):
                rospy.logerr("CircleVisionTarget - Cannot find the center of the circle because the vision message is too old")
            raise e
