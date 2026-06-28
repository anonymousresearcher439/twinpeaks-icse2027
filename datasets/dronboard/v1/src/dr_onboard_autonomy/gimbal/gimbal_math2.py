#!/usr/bin/env python3

import math
from typing import Tuple

import numpy as np
import tf

from numpy.linalg import norm
from tf.transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_conjugate,
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
    unit_vector,
    euler_from_quaternion,
    _EPS as small_number_threshold,
)

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.gimbal import AbstractGimbalCalculator, DroneData
from dr_onboard_autonomy.gimbal.data_types import Quaternion as QuaternionData


LlaTuple = Tuple[float, float, float]  # (latitude, longitude, altitude)
QuaternionTuple = Tuple[float, float, float, float]  # (x, y, z, w)


class GimbalCalculator2(AbstractGimbalCalculator):

    @staticmethod
    def find_track_position_attitude(drone_data: DroneData, stare_position: Lla) -> Tuple[float, float, float, float]:
        pos = drone_data.position
        drone_lla = pos.latitude, pos.longitude, pos.altitude

        attitude = drone_data.attitude
        orientation = attitude.x, attitude.y, attitude.z, attitude.w

        target_lla = stare_position.latitude, stare_position.longitude, stare_position.altitude
        return GimbalCalculator2.gimbal_angles4(drone_lla, orientation, target_lla)

    @staticmethod
    def _gimbal_angles3(
        drone_lat,
        drone_lon,
        drone_alt,
        drone_qt_x,
        drone_qt_y,
        drone_qt_z,
        drone_qt_w,
        target_lat,
        target_lon,
        target_alt,
    ):
        drone_lla = (drone_lat, drone_lon, drone_alt)
        drone_orientation = (drone_qt_x, drone_qt_y, drone_qt_z, drone_qt_w)
        target_lla = target_lat, target_lon, target_alt
        return GimbalCalculator2.gimbal_angles4(drone_lla, drone_orientation, target_lla)

    @staticmethod
    def gimbal_angles4(
        drone_lla: LlaTuple, orientation: QuaternionTuple, target_lla: LlaTuple
    ):
        # Find vector that points from the drone to the target in ENU where the drone is at the origin
        # the return value is a tuple with 4 floats like this: (e, n, u, 1)
        target_enu = GimbalCalculator2.find_a2b_enu(drone_lla, target_lla)

        # we can use this matrix to transform points in body frame to points in ENU
        # the starting frame (body in this case) is on the right because we'd put a vector in body
        # coordinates on the right when we multiply
        tx_ENU_body = quaternion_matrix(orientation)
        # we can use this matrix to transform points in ENU to points in body frame
        tx_body_ENU = quaternion_matrix(quaternion_conjugate(orientation))

        # target_body is a Forward, Left, Up, coordinate that describes where the target is
        target_body = tx_body_ENU.dot(target_enu)

        # we need the matrix that takes points from the body frame to the neutral gimbal frame
        # I named the neutral gimbal frame gimbal0 which is short for "gimbal-naught"
        # the gimbal uses forward, right, down coordinates
        # to get started we'll assume the origin of gimbal frame is coincident with the body frame
        # here we list the gimbal axes in terms of the body frame
        gimbal0_x = [1,  0,  0, 0]  # forward
        gimbal0_y = [0, -1,  0, 0]  # right
        gimbal0_z = [0,  0, -1, 0]  # down
        gimbal0_w = [0,  0,  0, 1]  # the last row

        # the transform converts coordinates in the body frame to coordinate in the neutral gimbal frame
        tx_gimbal0_body = np.array(
            [gimbal0_x, gimbal0_y, gimbal0_z, gimbal0_w], dtype=np.float64
        )
        # we find the target coordinates in the neutral gimbal frame
        target_gimbal0 = tx_gimbal0_body.dot(target_body)

        # now we need to find the transform that rotates gimbal0_x so it points at the target
        # we need a unit vector for our gimbal_final x axis
        # gimbalf is short for "gimbal final." It's the frame where the camera looks at the target
        gimbalf_x = GimbalCalculator2.unit_vector_from_pvector(target_gimbal0)
        v_cameraforward_gimbal0 = gimbalf_x.copy()
        
        # q_gimbalfx_gimbal0 is called that because it represents the rotation from gimbal0 to gimbalfx
        q_gimbalfx_gimbal0 = GimbalCalculator2.find_rotation_quaternion(gimbal0_x, gimbalf_x)
        q_gimbal0_gimbalfx = quaternion_conjugate(q_gimbalfx_gimbal0)

        # this transform takes points in gimbal0 and finds the corrosponding points
        # in the frame that has its x-axis pointing at the target
        tx_gimbalfx_gimbal0 = quaternion_matrix(q_gimbalfx_gimbal0)
        tx_gimbal0_gimbalfx = quaternion_matrix(q_gimbal0_gimbalfx)

        # to get the vector that points camera-right, we need to find:
        # right = down cross camera-forward 
        # where down points towards the "ground" and it's value is (0,0,-1) in ENU
        # so we need a transform that takes points in ENU and finds them in gimbal0
        # first lets make a vec4 for the down direction in ENU
        v_down_ENU = GimbalCalculator2.make_unit_vector(0, 0, -1)
        # next lets make a transform that takes points from ENU to gimbal0
        tx_gimbal0_ENU = tx_gimbal0_body.dot(tx_body_ENU)
        # now we can find down in gimbal0
        v_down_gimbal0 = tx_gimbal0_ENU.dot(v_down_ENU)
        # we find the direction of camera right in gimbal0
        v_cameraright_gimbal0 = GimbalCalculator2.unit_vector_from_cross_product(v_down_gimbal0, v_cameraforward_gimbal0)

        # now that we have camera-right, we can find camera down with:
        # camera-down = camera-forward cross camera-right
        v_cameradown_gimbal0 = GimbalCalculator2.unit_vector_from_cross_product(v_cameraforward_gimbal0, v_cameraright_gimbal0)

        # now we can make our 4x4 transform matrix for gimbal-final
        tx_gimbalf_gimbal0 = np.array(
            [
                v_cameraforward_gimbal0,
                v_cameraright_gimbal0,
                v_cameradown_gimbal0,
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )
        tx_gimbal0_gimbalf = tx_gimbalf_gimbal0.T


        # finally we can make a quaternion
        q_gimbalf_gimbal0 = quaternion_from_matrix(tx_gimbalf_gimbal0)
        q_gimbal0_gimbalf = quaternion_from_matrix(tx_gimbal0_gimbalf)
        # return q_gimbalf_gimbal0
        return q_gimbal0_gimbalf

    @staticmethod
    def find_a2b_enu(a_lla: LlaTuple, b_lla: LlaTuple):
        """find the East, North, and Up coordinates of the b described in the ENU coordinate system
        that has its origin at the a.
        """
        a = Lla(*a_lla)
        b = Lla(*b_lla)
        n, e, d = a.distance_ned(b)
        u = -1.0 * d
        return e, n, u, 1

    @staticmethod
    def unit_vector_from_cross_product(a, b):
        x = np.cross(a[:3], b[:3])
        return GimbalCalculator2.make_unit_vector(*x)


    @staticmethod
    def make_unit_vector(x, y, z):
        """make a vec4 backed by a numpy array like this: (x, y, z, 0)
        """
        result = np.zeros((4,), dtype=np.float64)
        result[0] = x
        result[1] = y
        result[2] = z
        return unit_vector(result)


    # when we take the dot product of two unit vectors
    # we end up with cos(the angle between them)
    # if the angle is nearly 0 then the value is close to 1.0
    # to ensure numerical stability, we have a threashold where we consider two vectors pointing in
    # the same direction if cos(the angle between them) is within this distance of 1.0
    # similarly if cos(the angle between them) is within this distance of -1 then we consider the
    # vectors as pointing in the opposite direction.
    #
    # This value corrosponds to an angle that is 1/100 degrees (or 1% of 1 degree).
    # At some point the gimbal can't move precisely enough for the difference to matter.
    # I'm guessing it can't rotate to within 1% of 1 degree.
    # at 1/100 degrees, the center of the image would be off by less than 2 meters when the target is
    # 10,000 meters away... Of course we could always change this if needed
    _same_direction_threshold = 1 - math.cos(math.radians(1 / 100))
    # tf.transformations uses there own value for the same idea
    _same_direction_threshold = small_number_threshold


    @staticmethod
    def find_rotation_quaternion(start_vec, target_vec):
        """Find the quaternion that rotates start_vec so it points at target_vec"""
        # based on approach from page 79 of Real-Time Rendering 3rd edition
        s = unit_vector(start_vec[0:3])
        t = unit_vector(target_vec[0:3])

        e = np.dot(s, t)

        # if e is 1 (or nearly so) then start_vec already points at target_vec
        # in this case return the identity quaternion
        if e > 0.0 and (1.0 - e) < GimbalCalculator2._same_direction_threshold:
            print("vectors point in the same direction")
            quaternion = np.zeros((4,), dtype=np.float64)
            quaternion[3] = 1.0
            return quaternion

        # if e is -1 then s = -1 * t. So we can rotate around any axis that's perpendicular to start_vec
        if e < 0.0 and e + 1.0 < GimbalCalculator2._same_direction_threshold:
            print("vectors point in opposite direction")
            rotation_axis = GimbalCalculator2._find_perpendicular_axis(s)
            return quaternion_about_axis(math.pi, rotation_axis)

        rotation_axis = np.cross(s, t)
        tmp = np.sqrt(2.0 * (1 + e))

        w = tmp / 2.0
        scaler = 1.0 / tmp
        v = rotation_axis * scaler

        quaternion = np.empty((4,), dtype=np.float64)
        quaternion[:3] = v[:]
        quaternion[3] = w
        return quaternion

    @staticmethod
    def _find_perpendicular_axis(vec):
        """find a vector that is perpendicular"""
        # this code is based on the formula on page 71 of Real-Time Rendering 3rd edition
        # we want a numerically stable way to find a perpendicular axis to rotate around
        #
        # 1. we find the value in vec that is nearest to 0. We set that value to 0.
        # 2. we swap the other two values
        # 3. we negate one of swapped values

        # 1
        smallest = GimbalCalculator2._smallest_index(vec)
        rotation_axis = np.copy(vec)
        rotation_axis[smallest] = 0.0

        # 2
        a = (smallest + 1) % 3
        b = (smallest + 2) % 3
        rotation_axis[a], rotation_axis[b] = rotation_axis[b], rotation_axis[a]

        # 3
        rotation_axis[a] *= -1

        return unit_vector(rotation_axis)

    @staticmethod
    def _smallest_index(vec):
        """find element of vec that is nearest to 0 and return its index"""
        smallest = 0
        vec_abs = np.abs(vec)
        for i in range(1, 3):
            if vec_abs[i] < vec_abs[smallest]:
                smallest = i
        return smallest

    @staticmethod
    def unit_vector_from_pvector(vec4):
        vec4 = np.copy(vec4)
        vec4[3] = 0.0
        return unit_vector(vec4)

    @staticmethod
    def quaternion_matrix_to_dest(q):
        """just like quaternion_matrix but the transformation takes you to the destination"""
        q = quaternion_conjugate(q)
        return quaternion_matrix(q)

    @staticmethod
    def printvec(v):
        print(np.round(v, 3))

    @staticmethod
    def quaternion_angle(q):
        w = q[3]
        return 2.0 * np.arccos(w)

    @staticmethod
    def quaternion_vec(q):
        v = q[:3]
        if np.any(v):
            return unit_vector(v)
        return v

    @staticmethod
    def print_quaternion(q):
        GimbalCalculator2.printvec(q)
        v = GimbalCalculator2.quaternion_vec(q)
        theta = GimbalCalculator2.quaternion_angle(q)
        print("    vector part:", np.round(v, 3))
        theta = np.degrees(theta)
        print("     angle part:", np.round(theta, 3))

    @staticmethod
    def attitude_drone_to_world_frame(
        gimbal_attitude: QuaternionData,
        drone_attitude: QuaternionData
    ) -> QuaternionData:
        """Transforms the provided gimbal attitude in the drone's frame of reference into the world
        frame of reference. Only transforms the gimbal yaw as the gimbal's pitch and roll are
        assumed already in the world frame.

        Args:
            gimbal_attitude: assumed x is axis aligned in the direction the drone faces, z goes out
                the top, and y is a positive 90 degree rotation around z from x
            drone_attitude: assumed in xyz in ENU world coordinates 
        """
        euler = euler_from_quaternion(drone_attitude.astuple(), 'szxy')
        '''
        remove any pitch and yaw rotations from drone to world transformation as gimbal quaternion 
        pitch and yaw are already in the world frame
        euler[0] is rotation about the z-axis since axes input for euler_from_quaternion is 'szxy'
        want 'szxy' so the full yaw component of the drone is in one axis
        '''
        drone_to_world_yaw_transform = quaternion_from_euler(0, 0, euler[0], axes='sxyz')
        '''
        quaternion multiplication order: applying global (drone to world) transformation to local
        (drone) gimbal attitude -> q_drone_world * q_gimbal_drone. Local attitude on the right and
        global on the left.
        If applying multiple local transforms, order left to right would be in 
        the desired order of the local transformations
        https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
        '''
        gimbal_attitude_world_frame = quaternion_multiply(
            (
                drone_to_world_yaw_transform[0],
                drone_to_world_yaw_transform[1],
                drone_to_world_yaw_transform[2],
                drone_to_world_yaw_transform[3]
            ),
            gimbal_attitude.astuple()
        )

        return QuaternionData(
            gimbal_attitude_world_frame[0],
            gimbal_attitude_world_frame[1],
            gimbal_attitude_world_frame[2],
            gimbal_attitude_world_frame[3]
        )


class SDHX10GimbalCalculator(AbstractGimbalCalculator):

    _LEFT_AXIS = (0.0, 1.0, 0.0)

    @staticmethod
    def find_track_position_attitude(drone_data: DroneData, stare_position: Lla) -> Tuple[float, float, float, float]:
        n, e, d = drone_data.position.distance_ned(stare_position)
        horizontal_distance = math.sqrt(n*n + e*e)
        pitch_angle = math.atan(d/horizontal_distance)
        return quaternion_about_axis(pitch_angle, SDHX10GimbalCalculator._LEFT_AXIS)
    
    @staticmethod
    def find_aircraft_yaw(drone_data: DroneData, stare_position: Lla) -> float:
        n, e, _ = drone_data.position.distance_ned(stare_position)
        return math.atan2(n, e)