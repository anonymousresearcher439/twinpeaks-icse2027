# bash to run this test
# export PYTHONPATH="$PYTHONPATH:$PWD/test:$PWD/src"
# pytest test/test_gimbal_manuever.py

import math
from typing import Tuple
import unittest

import numpy as np
from droneresponse_mathtools import Lla, geoid_height
from tf.transformations import (
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
    unit_vector,
    euler_from_quaternion,
    euler_matrix,
)

from dr_onboard_autonomy.gimbal.data_types import DroneData, Quaternion
from dr_onboard_autonomy.gimbal.gimbal_math2 import GimbalCalculator2, SDHX10GimbalCalculator
from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod

find_rotation_quaternion = GimbalCalculator2.find_rotation_quaternion
gimbal_angles4 = GimbalCalculator2.gimbal_angles4


def convert_to_gimbal_calc_inputs(pos: Tuple[float, float, float], drone_orientation:Tuple[float, float, float, float], target: Tuple[float, float, float]) -> Tuple[DroneData, Lla]:
    drone_pos = Lla(*pos)
    attitude = Quaternion(
        x=drone_orientation[0],
        y=drone_orientation[1],
        z=drone_orientation[2],
        w=drone_orientation[3],
    )
    drone = DroneData(position=drone_pos, attitude=attitude)
    target = Lla(*target)
    return drone, target


class TestGimbalMath(unittest.TestCase):
    def test_find_rotation_quaternion1(self):
        s = (1, 0, 0, 0)
        M = euler_matrix(0, 0, math.radians(90))

        t = M.dot(s)    

        q = find_rotation_quaternion(s, t)
        M2 = quaternion_matrix(q)
        t_actual = M2.dot(s)
        self.assertTrue(np.allclose(t, t_actual))

    def test_find_rotation_quaternion2(self):
        s = (1, 0, 0, 0)
        M = euler_matrix(math.radians(45), math.radians(45), math.radians(45))
        t = M.dot(s)

        q = find_rotation_quaternion(s, t)
        M_q = quaternion_matrix(q)
        t_q = M_q.dot(s)
        self.assertTrue(np.allclose(t, t_q))

    def test_find_rotation_quaternion_s_and_t_same_direction(self):
        s = np.array([1, 0, 0, 0], dtype=np.float64)
        t = np.copy(s)

        q = find_rotation_quaternion(s, t)
        q_identity = np.array([0, 0, 0, 1], dtype=np.float64)
        self.assertTrue(np.allclose(q_identity, q))

    def test_find_rotation_quaternion_s_and_t_same_direction2(self):
        s = np.array([1, 1, 0, 0], dtype=np.float64)
        s = s / math.sqrt(2)
        t = np.copy(s)

        q = find_rotation_quaternion(s, t)
        q_identity = np.array([0, 0, 0, 1], dtype=np.float64)
        self.assertTrue(np.allclose(q_identity, q))

    def test_find_rotation_quaternion_s_and_t_opposite_direction(self):
        s = np.array([1, 1, 0, 0], dtype=np.float64)
        s = s / math.sqrt(2)
        t = np.copy(s)
        t = t * -1.0

        q = find_rotation_quaternion(s, t)
        # in this case, the axis of rotation could be any perpendicular axis
        M = quaternion_matrix(q)
        t_actual = M.dot(s)
        self.assertTrue(np.allclose(t, t_actual))
    
    def test_gimbal_angles4_target_100_meters_south_drone_faces_north(self):
        # In this case, the drone is at white field.
        # the target is 100 meters south
        # the drone is facing north

        # first create test positions for drone and target
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=0, target_n=-100, target_u=0
        )
        # create orientation for drone facing north
        # in ENU we rotate 90 degrees around the up axis
        drone_orientation = quaternion_about_axis(math.radians(90), (0, 0, 1))
        drone_euler_orientation = euler_from_quaternion(drone_orientation)
        print("drone_euler_orientation")
        printeuler(drone_euler_orientation)

        drone_data, target_data = convert_to_gimbal_calc_inputs(drone_pos, drone_orientation, target_pos)
        q_gimbal0 = GimbalCalculator2.find_track_position_attitude(drone_data, target_data)
        q_gimal_correct = ChowdhuryMethod.find_track_position_attitude(drone_data=drone_data, stare_position=target_data)
        q_gimbal = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        self.assertTrue(np.allclose(q_gimbal0, q_gimbal))
        self.assertTrue(np.allclose(q_gimal_correct, q_gimbal0, atol=0.01))

        # expected is the gimbal is rotated 180 degrees around z
        q_gimbal_expected = quaternion_about_axis(math.radians(180), (0, 0, 1))
        # there are two quaternions that are equivelent to the same overall rotation
        # this is because a rotation around axis u by angle a
        # is the same as a rotation around -u by angle -a
        option1 = np.allclose(q_gimbal, q_gimbal_expected)
        option2 = np.allclose(q_gimbal, q_gimbal_expected * -1.0)

        self.assertTrue(option1 or option2)

    def test_gimbal_angles4_target_100_meters_east_drone_faces_north(self):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=0, target_u=0
        )
        # create orientation for drone facing north
        # in ENU we rotate 90 degrees around the up axis
        drone_orientation = quaternion_about_axis(math.radians(90), (0, 0, 1))
        drone_euler_orientation = euler_from_quaternion(drone_orientation)
        print("drone_euler_orientation")
        printeuler(drone_euler_orientation)

        q_gimbal = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # expect the gimbal to be rotated 90 degrees around z
        q_gimbal_expected = quaternion_about_axis(math.radians(90), (0, 0, 1))

        # there are two quaternions that are equivelent to the same overall rotation
        # this is because a rotation around any axis u by any angle a
        # is the same as a rotation around -u by angle -a
        option1 = np.allclose(q_gimbal, q_gimbal_expected)
        option2 = np.allclose(q_gimbal, q_gimbal_expected * -1.0)

        self.assertTrue(option1 or option2)

    def test_gimbal_angles4_target_100_meters_east_drone_faces_north_east(self):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=0, target_u=0
        )

        # create orientation for drone facing north
        # in ENU we rotate 90 degrees around the up axis
        drone_orientation = quaternion_about_axis(math.radians(45), (0, 0, 1))
        drone_euler_orientation = euler_from_quaternion(drone_orientation)
        print("drone_euler_orientation")
        printeuler(drone_euler_orientation)

        q_gimbal = gimbal_angles4(drone_pos, drone_orientation, target_pos)
        
        x = drone_orientation[0]
        y = drone_orientation[1]
        z = drone_orientation[2]
        w = drone_orientation[3]
        q = Quaternion(x, y, z, w)

        drone_data = DroneData(position=Lla(*drone_pos), attitude=q)
        target_lla = Lla(*target_pos)
        q_cm = ChowdhuryMethod.find_track_position_attitude(drone_data, target_lla)
        print("check CM results with our results")
        print("CM result")
        print_quaternion(q_cm)
        print("gimbal_angles4 result")
        print_quaternion(q_gimbal)

        cm_option1 = np.allclose(q_cm, q_gimbal)
        cm_option2 = np.allclose(q_cm, q_gimbal * -1.0)

        # expected is the gimbal is rotated 180 degrees around z
        q_gimbal_expected = quaternion_about_axis(math.radians(45), (0, 0, 1))

        # there are two quaternions that are equivelent to the same overall rotation
        # this is because a rotation around axis u by angle a
        # is the same as a rotation around -u by angle -a
        option1 = np.allclose(q_gimbal, q_gimbal_expected)
        option2 = np.allclose(q_gimbal, q_gimbal_expected * -1.0)

        self.assertTrue(option1 or option2)

    def test_when_target_is_100_meters_east_54_meters_north_12_meters_down_and_drone_faces_east(self):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392


        # create orientation for drone facing east
        # in ENU we rotate 0 degrees around the up axis
        drone_orientation = quaternion_about_axis(math.radians(0), (0, 0, 1))
        drone_euler_orientation = euler_from_quaternion(drone_orientation)
        print("drone_euler_orientation")
        printeuler(drone_euler_orientation)

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=54, target_u=-12
        )
        drone_data, target_data = convert_to_gimbal_calc_inputs(drone_pos, drone_orientation, target_pos)
        qc = ChowdhuryMethod.find_track_position_attitude(drone_data, target_data)
        qg = GimbalCalculator2.find_track_position_attitude(drone_data, target_data)

        for x_c, x_g in zip(qc, qg):
            self.assertAlmostEqual(x_c, x_g, 2)
        
        ec = euler_from_quaternion(qc)
        eg = euler_from_quaternion(qg)

        for a_c, a_g in zip(ec, eg):
            a_c = math.degrees(a_c)
            a_g = math.degrees(a_g)
            self.assertAlmostEqual(a_c, a_g, 0)

    def test_gimbal_angles4_target_100meters_south_and_100meters_east_and_141meters_up_when_drone_faces_north(
        self,
    ):
        # this test puts the target just has much up as it is horizontially
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat,
            drone_lon,
            drone_alt,
            target_e=100,
            target_n=-100,
            target_u=141.4213562373095,
        )
        # drone faces north
        drone_orientation = quaternion_about_axis(math.radians(90), (0, 0, 1))

        q_actual = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # first rotate 135 degrees around down (in neutral gimbal frame) so camera faces target in
        # the left-to-right direction. Then rotate 45 degrees up
        r1 = quaternion_about_axis(math.radians(135), (0, 0, 1))
        r2 = quaternion_about_axis(math.radians(45), (unit_vector((-1, -1, 0))))
        q_expect = quaternion_multiply(r2, r1)

        option1 = np.allclose(q_actual, q_expect)
        option2 = np.allclose(q_actual, q_expect * -1.0)

        self.assertTrue(option1 or option2)


    def test_gimbal_angles4_target_100meters_east_drone_faces_north_with_45_degree_roll(
        self,
    ):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=0, target_u=0
        )
        # drone faces north
        o1 = quaternion_about_axis(math.radians(90), (0, 0, 1))
        # from there rotate around north axis 45 degrees
        o2 = quaternion_about_axis(math.radians(45), (0, 1, 0))
        drone_orientation = quaternion_multiply(o2, o1)

        q_actual = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # to find the proper result, we undo the drone_orientation
        # then we point at the target
        # we need to undo the rotation in the gimbal0 frame
        # undo the roll
        r1 = quaternion_about_axis(math.radians(-45), (1, 0, 0))
        # It's like we've tilted our head to the right 45 degrees while facing north
        # The up direction is equal amounts left and up
        # but in forward-right-down, the left direction is y=-1 and the up direction is z=-1
        up_direction = unit_vector((0, -1, -1))
        # to go from north to east we go -90 degrees around up
        r2 = quaternion_about_axis(math.radians(-90), up_direction)
        # now we combine our rotations, first r1 then r2
        q_expect = quaternion_multiply(r2, r1)

        option1 = np.allclose(q_actual, q_expect)
        option2 = np.allclose(q_actual, q_expect * -1.0)

        self.assertTrue(option1 or option2)

    def test_gimbal_angles4_target_100meters_east_and_100_meters_up_drone_faces_north_with_45_degree_roll(
        self,
    ):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=0, target_u=100
        )
        # drone faces north
        o1 = quaternion_about_axis(math.radians(90), (0, 0, 1))
        # from there rotate around north axis 45 degrees
        o2 = quaternion_about_axis(math.radians(45), (0, 1, 0))
        drone_orientation = quaternion_multiply(o2, o1)

        drone_data, target_data = convert_to_gimbal_calc_inputs(drone_pos, drone_orientation, target_pos)
        q_0 = GimbalCalculator2.find_track_position_attitude(drone_data, target_data)
        q_expected0 = ChowdhuryMethod.find_track_position_attitude(drone_data, target_data)


        q_actual = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # TODO find out why these gimbal commands differ
        # self.assertTrue(np.allclose(q_0, q_expected0, atol=0.01), "GimbalCalculator2 doesn't match ChowdhuryMethod")

        # to find the proper result, we undo the drone_orientation
        # then we point at the target
        # we need to undo the rotation in the gimbal0 frame
        # undo the roll
        r1 = quaternion_about_axis(math.radians(-45), (1, 0, 0))
        # It's like we've tilted our head to the right 45 degrees while facing north
        # The up direction is equal amounts left and up
        # but in forward-right-down, the left direction is y=-1 and the up direction is z=-1
        up_direction = unit_vector((0, -1, -1))
        # to go from north to east we go -90 degrees around up
        r2 = quaternion_about_axis(math.radians(-90), up_direction)
        # now we need to pitch up 45 degrees
        camera_left_direction = unit_vector((1, 0, 0))
        r3 = quaternion_about_axis(math.radians(-45), camera_left_direction)
        # now we combine our rotations, first r1 then r2
        q_expect = quaternion_multiply(r2, r1)
        # then apply r3's rotation
        q_expect = quaternion_multiply(r3, q_expect)
        option1 = np.allclose(q_actual, q_expect)
        option2 = np.allclose(q_actual, q_expect * -1.0)

        self.assertTrue(option1 or option2)

    
    def test_sdhx10_gimbal_angles4_target_100meters_east_and_100_meters_up_drone_faces_north_with_45_degree_roll(
        self,
    ):
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat, drone_lon, drone_alt, target_e=100, target_n=0, target_u=100
        )
        # drone faces north
        o1 = quaternion_about_axis(math.radians(90), (0, 0, 1))
        # from there rotate around north axis 45 degrees
        o2 = quaternion_about_axis(math.radians(45), (0, 1, 0))
        drone_orientation = quaternion_multiply(o2, o1)

        drone_data, target_data = convert_to_gimbal_calc_inputs(drone_pos, drone_orientation, target_pos)
        q_0 = SDHX10GimbalCalculator.find_track_position_attitude(drone_data, target_data)
        q_expected0 = ChowdhuryMethod.find_track_position_attitude(drone_data, target_data)


        q_actual = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # TODO find out why these gimbal commands differ
        # self.assertTrue(np.allclose(q_0, q_expected0, atol=0.01), "GimbalCalculator2 doesn't match ChowdhuryMethod")

        # to find the proper result, we undo the drone_orientation
        # then we point at the target
        # we need to undo the rotation in the gimbal0 frame
        # undo the roll
        r1 = quaternion_about_axis(math.radians(-45), (1, 0, 0))
        # It's like we've tilted our head to the right 45 degrees while facing north
        # The up direction is equal amounts left and up
        # but in forward-right-down, the left direction is y=-1 and the up direction is z=-1
        up_direction = unit_vector((0, -1, -1))
        # to go from north to east we go -90 degrees around up
        r2 = quaternion_about_axis(math.radians(-90), up_direction)
        # now we need to pitch up 45 degrees
        camera_left_direction = unit_vector((1, 0, 0))
        r3 = quaternion_about_axis(math.radians(-45), camera_left_direction)
        # now we combine our rotations, first r1 then r2
        q_expect = quaternion_multiply(r2, r1)
        # then apply r3's rotation
        q_expect = quaternion_multiply(r3, q_expect)
        option1 = np.allclose(q_actual, q_expect)
        option2 = np.allclose(q_actual, q_expect * -1.0)

        self.assertTrue(option1 or option2)

    def test_gimbal_angles4_target_100meters_east_and_100meters_north_141_meters_up_drone_faces_north_west_and_pitched_up_35degrees_with_15_degree_roll(
        self,
    ):
        # this test tries to combine everything
        # the target has components in every ENU direction
        # the drone orientation has "roll," "pitch" and "yaw" components
        drone_lat = 41.71437875722079
        drone_lon = -86.24183715080551
        drone_alt = 220.7392

        drone_pos, target_pos = create_test_positions(
            drone_lat,
            drone_lon,
            drone_alt,
            target_e=100,
            target_n=100,
            target_u=141.4213562373095,
        )
        # drone faces north west
        o1 = quaternion_about_axis(math.radians(135), (0, 0, 1))
        # drone is pitched up 35 degrees so we rotate 35 degrees around the axis that points right
        # after o1 the drone is facing north-west so it's right is to the  north east
        # in ENU north east is equal parts east and equal parts north
        tmp_right = unit_vector((1, 1, 0))
        o2 = quaternion_about_axis(math.radians(35), tmp_right)
        # drone is rolled 45 degrees around it's forward axis
        # but I don't know where forward points so I will make the transform matrix and see where
        # forward points
        tmp_forward = quaternion_matrix(quaternion_multiply(o2, o1)).dot([1, 0, 0, 0])
        # from there rotate 45 degrees around forward
        o3 = quaternion_about_axis(math.radians(45), tmp_forward[:3])
        drone_orientation_tmp = quaternion_multiply(o2, o1)
        drone_orientation = quaternion_multiply(o3, drone_orientation_tmp)

        q_actual = gimbal_angles4(drone_pos, drone_orientation, target_pos)

        # find q_expected
        # To find the expected result, we undo the drone_orientation, then rotate towards the target
        # but all our vectors are in the gimbal's neutral axis
        # undo the roll
        r1 = quaternion_about_axis(math.radians(-45), (1, 0, 0))
        # undo the pitch
        # we need to rotate around the vector that points right after we undo to roll
        # since the gimbal frame is forward-right-down and we rolled 45 degrees
        # the right is equal parts right and up
        tmp_right = unit_vector((0, 1, -1))
        r2 = quaternion_about_axis(math.radians(-35), tmp_right)
        # now we undo the yaw
        # I don't know what direction up points so I will build a matrix to find out
        # lets find where up points in the neutral gimbal axis
        tmp_up = quaternion_matrix(quaternion_multiply(r2, r1)).dot((0, 0, -1, 0))
        # now we can rotate -135 degrees around up to get back to ENU
        r3 = quaternion_about_axis(math.radians(-135), tmp_up)
        # next lets adjust yaw so it points north east
        r4 = quaternion_about_axis(math.radians(45), tmp_up)
        # now we need to look up 45 degrees, so we need to know where right points in neutral gimbal
        # frame. But I don't know where right is, so I will make a transform to find it
        # q_tmp = r4*r3*r2*r1
        q_tmp = quaternion_multiply(
            r4, quaternion_multiply(r3, quaternion_multiply(r2, r1))
        )
        # now we take the right direction from q_tmp and find it in the gimbal neutral frame
        tmp_right = quaternion_matrix(q_tmp).dot([0, 1, 0, 0])
        # to look up we go 45 degrees around right axis
        r5 = quaternion_about_axis(math.radians(45), tmp_right)
        # now we need to multiply everything together to get q_expected
        q_expected = quaternion_multiply(r5, q_tmp)

        option1 = np.allclose(q_actual, q_expected)
        option2 = np.allclose(q_actual, q_expected * -1.0)

        self.assertTrue(option1 or option2)

    def test_gimbal_drone_to_world_frame(self):
        from dr_onboard_autonomy.gimbal.gimbal_math2 import GimbalCalculator2
        
        tests = [
            {
                "description": "Euler ZXY| gimbal:: z: +pi/2 radians, drone:: - z: +pi/2 x: +pi/4 radians",
                "gimbal_attitude": Quaternion(x=0.0, y=0.0, z=0.7071081, w=0.7071055),
                "drone_attitude": Quaternion(x=0.2705975, y=0.2705985, z=0.6532827, w=0.6532803),
                "gimbal_world_attitude": Quaternion(x=0.0, y=0.0, z=1.0, w=-3.67696E-6)
            },
            {
                "description": "Euler ZXY| gimbal:: y: 0.2929552 radians, drone:: - z: -2.6748058 y: -0.0352246 x: 0.0079145 radians",
                "gimbal_attitude": Quaternion(x=0.0, y=0.1459544, z=0.0, w=0.9892913),
                "drone_attitude": Quaternion(x=-0.0162187, y=-0.0079225, z=-0.9727448, w=0.2311748),
                "gimbal_world_attitude": Quaternion(x=0.142005, y=0.0337238, z=-0.962521, w=0.228583)
            },
        ]
        
        for test in tests:
            with self.subTest(test["description"]):
                gimbal_quaternion_world_frame = GimbalCalculator2.attitude_drone_to_world_frame(
                    gimbal_attitude=test["gimbal_attitude"],
                    drone_attitude=test["drone_attitude"],
                )
                '''
                expect gimbal quaternion in the world frame to be rotated by only +pi about z-axis
                '''
                self.assertAlmostEqual(
                    gimbal_quaternion_world_frame.x, test["gimbal_world_attitude"].x, 5
                )
                self.assertAlmostEqual(
                    gimbal_quaternion_world_frame.y, test["gimbal_world_attitude"].y, 5
                )
                self.assertAlmostEqual(
                    gimbal_quaternion_world_frame.z, test["gimbal_world_attitude"].z, 5
                )
                self.assertAlmostEqual(
                    gimbal_quaternion_world_frame.w, test["gimbal_world_attitude"].w, 5
                )

def printeuler(e):
    print(np.round(np.degrees(e), 3))


def printvec(v):
    print(np.round(v, 3))


def quaternion_angle(q):
    w = q[3]
    return 2.0 * np.arccos(w)


def quaternion_vec(q):
    angle = quaternion_angle(q)
    v = q[:3]
    return v / np.sin(angle / 2)


def print_quaternion(q):
    printvec(q)
    v = quaternion_vec(q)
    theta = quaternion_angle(q)
    print("    vector part:", np.round(v, 3))
    theta = np.degrees(theta)
    print("     angle part:", np.round(theta, 3))


def asml_2_wgs84(lat, lon, amsl):
    return geoid_height(lat, lon) + amsl


def wgs84_2_amsl(lat, lon, wgs84):
    return wgs84 - geoid_height(lat, lon)


def create_test_positions(
    drone_lat, drone_lon, drone_amsl, target_e, target_n, target_u
):
    drone_wgs84_alt = asml_2_wgs84(drone_lat, drone_lon, drone_amsl)
    dlat, dlon, dalt = drone_lat, drone_lon, drone_wgs84_alt
    drone_result = drone_lat, drone_lon, drone_wgs84_alt
    target_lla = Lla(dlat, dlon, dalt).move_ned(target_n, target_e, -target_u)
    target_result = target_lla.lat, target_lla.lon, target_lla.alt
    return drone_result, target_result
