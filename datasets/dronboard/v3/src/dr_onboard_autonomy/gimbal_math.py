#!/usr/bin/env python3
from typing import Tuple
import math

from pymavlink import mavutil
from std_msgs.msg import Float64
import numpy as np
import tf
import rospy
from droneresponse_mathtools import Lla, geoid_height


from dr_onboard_autonomy.gimbal import AbstractGimbalCalculator, DroneData, Quaternion


class ChowdhuryMethod(AbstractGimbalCalculator):

    class RosCompassMsg:
        def __init__(self, value):
            self.data = value
    
    @staticmethod
    def enu_up_angle_to_compass(euler_angles_deg: float) -> float:
        x = 90 - euler_angles_deg
        return x % 360
        
    @staticmethod
    def find_track_position_attitude(drone_data: DroneData, stare_position: Lla) -> Tuple[float, float, float, float]:
        drone_pos = drone_data.position
        drone_attitude = drone_data.attitude
        target = stare_position

        quaternion_angles = (
            drone_attitude.x,
            drone_attitude.y,
            drone_attitude.z,
            drone_attitude.w,
        )
        euler_angles = tf.transformations.euler_from_quaternion(quaternion_angles)
        euler_angle = math.degrees(euler_angles[2])
        compass_angle = ChowdhuryMethod.enu_up_angle_to_compass(euler_angle)
        compass_hdg = ChowdhuryMethod.RosCompassMsg(compass_angle)

        roll, pitch, yaw = ChowdhuryMethod.gimbal_angles(
            drone_pos.latitude,
            drone_pos.longitude,
            drone_pos.altitude,
            drone_attitude.x,
            drone_attitude.y,
            drone_attitude.z,
            drone_attitude.w,
            target.latitude,
            target.longitude,
            target.altitude,
            compass_hdg,
        )
        
        # roll = AbstractGimbalCalculator.limit(roll, -40.0, 40.0)
        # pitch = AbstractGimbalCalculator.limit(pitch, -90.0, 90.0)
        # yaw = AbstractGimbalCalculator.limit(yaw, -180.0, 180.0)
        
        roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
        
        return tf.transformations.quaternion_from_euler(roll, pitch, yaw)


    @staticmethod
    def gimbal_angles(
        drone_lat,
        drone_long,
        drone_alt,
        drone_qt_x,
        drone_qt_y,
        drone_qt_z,
        drone_qt_w,
        spot_lat,
        spot_long,
        spot_alt,
        compass_hdg,
    ):  # spot is the GPS coordinate to look at
        gimbal_roll = 0
        gimbal_pitch = 0
        gimbal_yaw = 0

        quaternion_angles = (drone_qt_x, drone_qt_y, drone_qt_z, drone_qt_w)
        euler_angles = tf.transformations.euler_from_quaternion(quaternion_angles)
        roll = math.degrees(euler_angles[0])
        pitch = math.degrees(euler_angles[1])
        yaw = math.degrees(euler_angles[2])

        # gimbal_roll=gimbal_roll-roll #subtracting the drone's orientation from the gimbal's
        # gimbal_pitch=gimbal_pitch-pitch
        # gimbal_yaw=gimbal_yaw-yaw

        drone_lat_rad = math.radians(drone_lat)
        drone_long_rad = math.radians(drone_long)
        spot_lat_rad = math.radians(spot_lat)
        spot_long_rad = math.radians(spot_long)

        alt_distance = abs(drone_alt - spot_alt)

        coords_1 = (drone_lat, drone_long, drone_alt)
        coords_2 = (spot_lat, spot_long, spot_alt)

        geodesic_distance = ChowdhuryMethod.horizontal_distance(Lla(*coords_1), Lla(*coords_2))

        angle = ChowdhuryMethod.angle_from_coordinate(
            drone_lat_rad, drone_long_rad, spot_lat_rad, spot_long_rad
        )

        # print("angle difference", angle)

        # To-Do, divide geodestic by alt_distance and subtract it from -45 pitch

        # alt_adjustment=(alt_distance-20)/10

        # pitch_angle=abs(alt_adjustment)

        # ratio=(alt_distance/geodesic_distance)

        theta = math.degrees(math.atan2(alt_distance, geodesic_distance))

        x = 180 - theta - 90

        pitch_angle = 90 - x

        # print(pitch_angle)

        gimbal_pitch = -pitch_angle

        gimbal_yaw = angle - compass_hdg.data

        # print("compass data", compass_hdg.data)

        return (gimbal_roll, gimbal_pitch, gimbal_yaw)
        # return (roll, pitch, yaw)

    @staticmethod
    def angle_from_coordinate(lat1, long1, lat2, long2):
        dLon = long2 - long1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dLon)

        brng = math.atan2(y, x)

        brng = math.degrees(brng)
        brng = (brng + 360) % 360

        return brng  # return bearing

    @staticmethod
    def create_vec3_in_camera_coordinates(x, y, x_res, y_res):
        # we need to find a new x and y in a different image coordinate system
        # x goes from 0 to 1 where 0 is the left and 1 is the right of the image
        # y goes from 0 to 1 where 0 is the top and 1 is the bottom of the image
        # we find a new coordinate where 0,0 is the center
        # x goes from -0.5 all the way at the left and +0.5 all the way to right
        # y goes from -0.5 all the way at the bottom and +0.5 all the way to top

        x = x / x_res  # normalization
        y = y / y_res

        x_result = x - 0.5
        y_result = -y + 0.5
        x, y = x_result, y_result

        # now we multiply x by hfov to get theta this is our horizontal angle in the camera body
        # frame in spherical coordinate
        hfov = 2.0
        vfov = 1.44862
        theta = x * hfov

        # we can do something similar to get phi, which is our vertical angle in spherical coordinates
        phi = y * vfov

        # now to get Cartesian coordinates in the body frame of the camera
        x_C = math.sin(phi) * math.cos(theta)
        y_C = math.sin(theta) * math.sin(phi)
        z_C = math.cos(phi)
        # we have unit vector that points from the
        # camera to the object in the camera frame
        return (x_C, y_C, z_C)

    @staticmethod
    def calculate_gimbal_setpoint():  # will be needed if we want to change drone's flight path
        pass

    @staticmethod
    def horizontal_distance(a: Lla, b: Lla):
        d = a.distance_ned(b)
        d[2] = 0.0
        return np.sqrt(d.dot(d))
