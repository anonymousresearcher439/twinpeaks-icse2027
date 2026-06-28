import json
from math import floor
from queue import Queue
from typing import Dict, Tuple

import rospy

from dr_onboard_autonomy.gimbal.data_types import Quaternion as QuaternionData
from dr_onboard_autonomy.models import LlaPosition, Quaternion

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    briarlla_from_lat_lon_alt,
    find_circle_start,
    LlaDict
)
from dr_onboard_autonomy.gimbal.geolocation import geolocate_object_from_camera
from dr_onboard_autonomy.gimbal.gimbal_math2 import GimbalCalculator2
from dr_onboard_autonomy.states import BaseState, BriarHover, BriarWaypoint
from dr_onboard_autonomy.states.BriarCircle import BriarCircle
from dr_onboard_autonomy.states.ReadDroneSensors import ReadMessagesAirborne
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


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


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
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
        **kwargs
    ):
        kwargs["outcomes"] = self._process_outcomes(kwargs, ["succeeded_circle"], STANDARD_TRANSITIONS_FLYING.keys())
        super().__init__(**kwargs)
        self._target_circle_radius = target_circle_radius
        self._target_circle_height = target_circle_height
        self._target_approach_speed = target_approach_speed
        self._circle_speed = circle_speed
        self._messages_return_channel = Queue()
        self._read_messages = ReadMessagesAirborne(
            message_names=["vision"],
            return_channel=self._messages_return_channel,
            **kwargs
        )
        self._read_drone_data = ReadMessagesAirborne(
            message_names=["position", "imu"],
            return_channel=self._messages_return_channel,
            **kwargs
        )
        self._pass_through_kwargs = kwargs


    def on_entry(self, userdata):
        briar_hover = BriarHover(hover_time=5.0, start_position=None, **self._pass_through_kwargs)
        hover_outcome = self.execute_substate(briar_hover, userdata)
        if hover_outcome != "succeeded_hover":
            return hover_outcome

        outcome = self.execute_substate(self._read_messages, userdata)
        if outcome != "succeeded":
            return outcome

        vision_data: Dict = json.loads(self._messages_return_channel.get()["vision"].payload)
        ts = vision_data['ts']

        # if ts is newer than drone pos or attitude
        t_pos_newest, _ = self.drone.data.location.last()
        t_att_newest, _ = self.drone.data.attitude.last()
        if ts > t_pos_newest or ts > t_att_newest:
            outcome = self.execute_substate(self._read_drone_data, userdata)
            if outcome != "succeeded":
                return outcome

        try:
            drone_pos, drone_quaternion, gimbal_quaternion = self._find_frames(ts)

        except:
            return "error"

        gimbal_attitude_world = GimbalCalculator2.attitude_drone_to_world_frame(
            gimbal_attitude=QuaternionData(
                gimbal_quaternion.x,
                gimbal_quaternion.y,
                gimbal_quaternion.z,
                gimbal_quaternion.w,
            ),
            drone_attitude=QuaternionData(
                drone_quaternion.x,
                drone_quaternion.y,
                drone_quaternion.z,
                drone_quaternion.w,
            )
        )

        drone_pos = drone_pos.to_wgs84_ellipsoid()
        '''
        fov_h used below is specific to the IMX477
        ideally would be populated by a config specific to the UAV
        '''
        target_location: BriarLla = briarlla_from_lat_lon_alt(
            *(
                *geolocate_object_from_camera(
                    fov_h=80,
                    lla=(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude),
                    image_res=(vision_data["x_res"], vision_data["y_res"]),
                    target_coords=(vision_data["x"], vision_data["y"]),
                    quaternion_gimbal=gimbal_attitude_world.astuple(),
                    ground_alt=self.drone.data.arm_position.altitude
                ),
                False
            )
        )

        self.mqtt_client.publish("target_position",
            {
                "latitude": target_location.amsl.lla.latitude,
                "longitude": target_location.amsl.lla.longitude,
                "altitude_amsl": target_location.amsl.lla.altitude
            }
        )

        rospy.loginfo(f"\nCircleVisionTarget - target_location @@@ {target_location.amsl.lla}\n")

        current_pos = self.drone.data.location.get_position()
        current_pos = current_pos.to_wgs84_ellipsoid()
        current_pos_briar_lla = BriarLla(LlaDict(
            latitude=current_pos.latitude,
            longitude=current_pos.longitude,
            altitude=current_pos.altitude
        ))
        circle_start: BriarLla = find_circle_start(
            current_pos=current_pos_briar_lla.ellipsoid.lla,
            target_pos=target_location.ellipsoid.lla,
            target_radius=self._target_circle_radius,
            target_circle_height=self._target_circle_height
        )

        circle_start_waypoint = BriarWaypoint(
            waypoint=circle_start.amsl.dict,
            stare_position=target_location.amsl.dict,
            speed=self._target_approach_speed,
            **self._pass_through_kwargs
        )
        circle_start_outcome = self.execute_substate(circle_start_waypoint, userdata)
        if circle_start_outcome != "succeeded_waypoints":
            return circle_start_outcome

        circle = BriarCircle(
            center_position=target_location.amsl.dict,
            stare_position=target_location.amsl.dict,
            speed=self._circle_speed,
            **self._pass_through_kwargs
        )
        circle_outcome = self.execute_substate(circle, userdata)
        if circle_outcome != "succeeded_circle":
            return circle_outcome

        return "succeeded_circle"


    def _find_frames(self, ts: float) -> Tuple[LlaPosition, Quaternion, Quaternion]:
        """
        """
        t_gimbal_newest, _ = self.drone.gimbal.attitude.last()
        try:
            drone_pos = self.drone.data.location.lookup_position(ts)
            drone_quaternion = self.drone.data.attitude.lookup_attitude(ts)
            if ts > t_gimbal_newest:
                _, gimbal_quaternion = self.drone.gimbal.attitude.last()
            else:
                gimbal_quaternion = self.drone.gimbal.attitude.lookup_attitude(ts)

            return (drone_pos, drone_quaternion, gimbal_quaternion)

        except Exception as e:
            t_att_oldest, _ = self.drone.data.attitude[0]
            t_pos_oldest, _ = self.drone.data.location[0]
            t_gimbal_oldest, _ = self.drone.gimbal.attitude[0]
            print(f"ts: {ts}, t_att_oldest: {t_att_oldest}, t_pos_oldest: {t_pos_oldest}, t_gimbal_oldest: {t_gimbal_oldest}")
            print(e)

            if ts <= max(t_att_oldest, t_pos_oldest, t_gimbal_oldest):
                rospy.logerr("CircleVisionTarget - Cannot find the center of the circle because the vision message is too old")
            raise e
