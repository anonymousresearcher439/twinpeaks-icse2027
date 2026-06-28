import math
import unittest
import numpy as np
import tempfile
import os
from tf.transformations import (
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
)

from dr_onboard_autonomy.models import DRConfig, Quaternion
from dr_onboard_autonomy.models.config import GimbalLimits
from dr_onboard_autonomy.gimbal.geolocation import (
    TwoAxisGimbalQuatCalculator,
    TwoAxisGimbalConfig,
    GimbalLimits
)


from .mock_types import mock_drone_config


ROLL_180 = quaternion_about_axis(math.radians(180), [1, 0, 0])


'''
"FOVh":80,
            "LLA":[41.687970, -86.249996, 100],
            "Image Res":[1080,200],
            "Target Coors":[350,200],"Quaternion":[0,0,0,0]
'''
class TestGeolocation(unittest.TestCase):
    def test_geolocate_object_from_camera(self):
        from dr_onboard_autonomy.gimbal.geolocator import GeoLocator

        '''
        target_object_lla calculated using droneresponse_mathtools Lla.move_ned
        quaternion calculated with:
        https://www.andre-gaschler.com/rotationconverter/
        quaternion rotation validated with:
        https://www.wolframalpha.com/input?i=draw+0.9238795%2B0i-0.3826834j%2B0k+as+a+rotation+operator
        '''
        bvh_tree = None
        test_cases = [
            {
                "description": "camera looks 45 degrees down and to the east with object centered",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (100, 100),
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 0.0
                },
                "target_object_lla": (0.0, 0.0004491576420505598, 0.0)
            },
            {
                "description": "camera looks 45 degrees down and to the north with object centered",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (100, 100),
                    "quaternion_gimbal": (-0.270598, 0.2705981, 0.6532814, 0.6532816),
                    "ground_alt": 0.0
                },
                "target_object_lla": (0.0004521847391196666, 0.0, 0.0)
            },
            {
                "description": "camera looks 45 degrees down and to the east with object right",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (150, 100),
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 0.0
                },
                "target_object_lla": (-0.0002682965799961742, 0.0004491576420505598, 0.0)
            },
            {
                "description": "camera looks 45 degrees down and to the east with object right and non-zero ground",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (150, 100),
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 20.0
                },
                "target_object_lla": (-0.0001609777671245695, 0.0002694937401791448, 20.0000955583)
            }
        ]

        for test in test_cases:
            with self.subTest(msg=test["description"]):
                gl = GeoLocator(test["inputs"]["lla"])
                gl.bvh_tree = bvh_tree
                target_lla = gl.geolocate_object_from_camera(
                    fov_h=test["inputs"]["fov_h"],
                    lla=test["inputs"]["lla"],
                    image_res=test["inputs"]["image_res"],
                    target_coords=test["inputs"]["target_coords"],
                    quaternion_gimbal=test["inputs"]["quaternion_gimbal"],
                    ground_alt=test["inputs"]["ground_alt"]
                )

                self.assertAlmostEqual(target_lla[0], test["target_object_lla"][0])
                self.assertAlmostEqual(target_lla[1], test["target_object_lla"][1])
                self.assertAlmostEqual(target_lla[2], test["target_object_lla"][2], 3)



class TestTerrainFileLoader(unittest.TestCase):
    def test_find_terrain_file(self):
        from dr_onboard_autonomy.gimbal.geolocation_boundingvols import find_terrain_file
        from droneresponse_mathtools import Lla
        from pathlib import Path

        # Create a temporary directory -> deleted once outside of with block
        with tempfile.TemporaryDirectory() as temp_dir:
            dir_name = temp_dir

            # Create two files inside the directory: peppermint and skyway
            with open(os.path.join(dir_name, "skyway.json"), "w") as f:
                f.write("[[36.218026144086004, -96.00916243678984, 169.38082099155173]]")
            with open(os.path.join(dir_name, "peppermint.json"), "w") as f:
                f.write("[[41.60915600000267, -86.36687500000191, 188.6226542062871]]")


            test_cases = [
                {
                    "description": "Finding skyway file",
                    "home_location": Lla(36.214367381762756, -96.00234965798748, 186.9179125345748),
                    "filepath": Path(os.path.join(dir_name, "skyway.json"))
                },
                {
                    "description": "Finding peppermint file",
                    "home_location": Lla(41.602507157849786, -86.35446298107279, 194.6243088030791),
                    "filepath": Path(os.path.join(dir_name, "peppermint.json"))
                }
            ]

            json_files = [file for file in Path(dir_name).rglob("*.json") if file.is_file()]

            for test in test_cases:
                with self.subTest(msg=test["description"]):
                    self.assertEqual(
                        find_terrain_file(test['home_location'], json_files),
                        test['filepath']
                    )

    def test_init_tree(self):
        from dr_onboard_autonomy.gimbal.geolocation_boundingvols import init_tree
        from pathlib import Path

        with tempfile.TemporaryDirectory() as temp_dir:
            dir_name = temp_dir
            json_filepath = Path(os.path.join(dir_name, "skyway.json"))
            joblib_filepath = Path(os.path.join(dir_name, "skyway.joblib.gz"))

            # Create sample file for failed loading
            with open(json_filepath, "w") as f:
                f.write("[foo]")
            with open(joblib_filepath, "w") as f:
                f.write("[bar]")


            test_cases = [
                {
                    "description": "Handing errors on loading from json file",
                    "filepath": json_filepath,
                    "use_cache": False,
                    "bvh_tree": None
                },
                {
                    "description": "Handing errors loading from joblib",
                    "filepath": joblib_filepath,
                    "use_cache": True,
                    "bvh_tree": None
                }
            ]


            for test in test_cases:
                with self.subTest(msg=test["description"]):
                    self.assertEqual(
                        init_tree(test['filepath'], test['use_cache']),
                        test['bvh_tree']
                    )


class TestTwoAxisGimbalFrame(unittest.TestCase):
    def test_init(self):
        c = TwoAxisGimbalQuatCalculator(
            configuration="roll_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )

        self.assertEqual(c.configuration, TwoAxisGimbalConfig.ROLL_FIRST)
        self.assertEqual(c.pitch_limits.min, -127)
        self.assertEqual(c.pitch_limits.max, 127)
        self.assertEqual(c.roll_limits._rad_min, math.radians(-60))
        self.assertEqual(c.roll_limits._rad_max, math.radians(60))
        self.assertEqual(c.pitch_limits._rad_min, math.radians(-127))
        self.assertEqual(c.pitch_limits._rad_max, math.radians(127))

    def test_build_from_dr_config(self):
        config: DRConfig = mock_drone_config()
        config.gimbal.axes = "roll_first"
        config.gimbal.pitch_limits = GimbalLimits(min=-127, max=127)
        config.gimbal.roll_limits = GimbalLimits(min=-60, max=60)

        c = TwoAxisGimbalQuatCalculator.build_from_config(config)
        self.assertEqual(c.configuration, TwoAxisGimbalConfig.ROLL_FIRST)
        self.assertEqual(c.pitch_limits.min, -127)
        self.assertEqual(c.pitch_limits.max, 127)
        self.assertEqual(c.roll_limits._rad_min, math.radians(-60))
        self.assertEqual(c.roll_limits._rad_max, math.radians(60))
        self.assertEqual(c.pitch_limits._rad_min, math.radians(-127))
        self.assertEqual(c.pitch_limits._rad_max, math.radians(127))

    def test_math1(self):
        two_axis_calc = TwoAxisGimbalQuatCalculator(
            configuration="roll_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )

        yaw = quaternion_about_axis(math.radians(90), [0, 0, 1])
        pitch = quaternion_about_axis(math.radians(45), [0, 1, 0])
        roll = quaternion_about_axis(math.radians(0), [1, 0, 0])

        # drone is facing North and pitched down 45 degrees
        drone_attitude = quaternion_multiply(yaw, quaternion_multiply(pitch, roll))

        is_ok, output = two_axis_calc.find_gimbal_quat(drone_attitude)
        self.assertTrue(is_ok) # this is true because the we didn't exceed the limits
        # the output should be yaw 90, pitch 0, but upside down (because the gimbal is Forward-Right-Down)
        # the two_axis_calc will
        # roll until the pitch axis is level
        # pitch until the forward axis is level
        # then roll 180 degrees to align the X-Y-Z axes
        expected = quaternion_multiply(yaw, ROLL_180)
        self.assertAlmostEqual(output[0], expected[0])
        self.assertAlmostEqual(output[1], expected[1])
        self.assertAlmostEqual(output[2], expected[2])
        self.assertAlmostEqual(output[3], expected[3])

    def test_math2(self):
        c = TwoAxisGimbalQuatCalculator(
            configuration="roll_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )

        yaw = quaternion_about_axis(math.radians(90), [0, 0, 1])
        pitch = quaternion_about_axis(math.radians(45), [0, 1, 0])
        roll = quaternion_about_axis(math.radians(10), [1, 0, 0])

        drone_attitude = quaternion_multiply(yaw, quaternion_multiply(pitch, roll))

        is_ok, output = c.find_gimbal_quat(drone_attitude)
        self.assertTrue(is_ok) # this is true because the we didn't exceed the limits
        # the output should be yaw 90 and pitch 0
        # the gimbal will
        # roll until the pitch axis is level
        # pitch until the forward axis is level
        expected = quaternion_multiply(yaw, ROLL_180)
        self.assertAlmostEqual(output[0], expected[0])
        self.assertAlmostEqual(output[1], expected[1])
        self.assertAlmostEqual(output[2], expected[2])
        self.assertAlmostEqual(output[3], expected[3])

    def test_math3(self):
        c = TwoAxisGimbalQuatCalculator(
            configuration="pitch_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )

        yaw = quaternion_about_axis(math.radians(90), [0, 0, 1])
        roll = quaternion_about_axis(math.radians(45), [1, 0, 0])
        pitch = quaternion_about_axis(math.radians(10), [0, 1, 0])

        drone_attitude = quaternion_multiply(yaw, quaternion_multiply(roll, pitch))

        is_ok, output = c.find_gimbal_quat(drone_attitude)
        self.assertTrue(is_ok) # this is true because the we didn't exceed the limits
        # the output should be yaw 90 and pitch 0
        # the gimbal will
        # roll until the pitch axis is level
        # pitch until the forward axis is level
        expected = quaternion_multiply(yaw, ROLL_180)
        self.assertAlmostEqual(output[0], expected[0])
        self.assertAlmostEqual(output[1], expected[1])
        self.assertAlmostEqual(output[2], expected[2])
        self.assertAlmostEqual(output[3], expected[3])


    def test_math4(self):
        """In this test we will make a roll_first and pitch first gimbal frame calculate.

        This test should show there is a difference between the two gimbal frames.

        Given a drone attitude such that:
        1. we roll -45 degrees
        2. we pitch -89.9999 degrees

        We will verify the roll_first gimbal frame by checking if the drone is looking north, and up is up.
        We will verify the pitch_first gimbal frame by checking if the drone is looking east, and up is up.
        """
        roller = TwoAxisGimbalQuatCalculator(
            configuration="roll_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )
        pitcher = TwoAxisGimbalQuatCalculator(
            configuration="pitch_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )


        roll = quaternion_about_axis(math.radians(-45), [1, 0, 0])
        pitch = quaternion_about_axis(math.radians(-89.9999), [0, 1, 0])
        yaw = quaternion_about_axis(math.radians(0), [0, 0, 1])
        drone_attitude = quaternion_multiply(yaw, quaternion_multiply(roll, pitch))

        # now in the drone body frame x is in the y-z plane (almost)
        # if we do roll first then we will end up looking north
        roller_success, roller_quat = roller.find_gimbal_quat(drone_attitude)
        # we expect roller_success to be False since we're basically
        # rolled almost 90 degrees but our limit is 60 degrees
        self.assertFalse(roller_success)
        # check the roller quaternion to see if we're nearly looking north
        # we will do this by finding the direction of the x-axis in the roller frame (ENU)
        # and comparing it to the expected direction (facing north)
        roller_matrix = quaternion_matrix(roller_quat)
        x = np.array([1, 0, 0, 1])
        x_roller = np.dot(roller_matrix, x)
        # make sure we are less than 0.01 degrees off from looking north
        expected_x_roller = np.array([0, 1, 0, 1]) # if we consider the gimbal frame in ENU this is the direction of the x axis
        # x_roller and expected_x_roller are both unit vectors so we use
        # the simplified dot product formula to get the angle between
        # them:
        #
        # cos(theta) = x_roller dot expected_x_roller
        # theta = acos(x_roller dot expected_x_roller)
        angle_from_north = angle_between(x_roller, expected_x_roller)
        self.assertLess(angle_from_north, math.radians(0.01), msg="The x-axis in the roll_frist gimbal frame isn't facing north")
        # next make sure z is pointing down in the roller frame
        z = np.array([0, 0, 1, 1])
        z_roller = np.dot(roller_matrix, z)
        expected_z_roller = np.array([0, 0, -1, 1])
        angle_from_up = angle_between(z_roller, expected_z_roller)
        self.assertLess(angle_from_up, math.radians(0.0001), msg="angle_from_up is not less than 0.0001")

        # now let's check the pitch first gimbal frame
        pitcher_success, pitcher_quat = pitcher.find_gimbal_quat(drone_attitude)
        # With the pitch first gimbal we should end looking east
        # let's see if x is almost facing east
        pitcher_matrix = quaternion_matrix(pitcher_quat)
        x_pitcher = np.dot(pitcher_matrix, x)
        expected_x_pitcher = np.array([1, 0, 0, 1]) # we're in ENU so x is east
        angle_from_east = angle_between(x_pitcher, expected_x_pitcher)
        self.assertLess(angle_from_east, math.radians(0.001), msg="The x-axis in the pitch_first gimbal frame isn't facing east")

        # now let's check the z-axis for the pitch first gimbal frame
        z_pitcher = np.dot(pitcher_matrix, z)
        # This is the expected z-axis in the pitch first gimbal frame if we were to consider it in ENU
        expected_z_pitcher = np.array([0, 0, -1, 1]) # we're in ENU but the gimbal is Forward-Right-Down so z is down
        self.assertAlmostEqual(expected_z_pitcher[0], z_pitcher[0])
        self.assertAlmostEqual(expected_z_pitcher[1], z_pitcher[1])
        self.assertAlmostEqual(expected_z_pitcher[2], z_pitcher[2])
        angle_from_up = angle_between(z_pitcher, expected_z_pitcher)
        self.assertLess(angle_from_up, math.radians(0.0001), msg="angle_from_up is not less than 0.0001")

        expected_pitcher_quat = quaternion_about_axis(math.radians(180), [1, 0, 0]) # we're looking east but upside down

        # last let's just check if the quaternions are equal
        self.assertQuatsAlmostEqual(pitcher_quat, expected_pitcher_quat, msg="pitcher_quat not equal to expected_pitcher_quat")

    def assertQuatsAlmostEqual(self, list1, list2, tol=1e-5, msg=None):
        def quaternion_to_array(q):
            return np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        if isinstance(list1, Quaternion):
            list1 = quaternion_to_array(list1)
        if isinstance(list2, Quaternion):
            list2 = quaternion_to_array(list2)
        if not np.allclose(list1, list2, atol=tol, rtol=0):
            raise AssertionError(f"Quats not almost equal:\nList1: {list1}\nList2: {list2} | {msg}")

    def test_math5(self):
        roller = TwoAxisGimbalQuatCalculator(
            configuration="roll_first",
            pitch_limits=GimbalLimits(min=-127, max=127),
            roll_limits=GimbalLimits(min=-60, max=60)
        )

        roll = quaternion_about_axis(math.radians(-45), [1, 0, 0])
        pitch = quaternion_about_axis(math.radians(-89.99), [0, 1, 0])
        yaw = quaternion_about_axis(math.radians(0), [0, 0, 1])
        drone_attitude = quaternion_multiply(yaw, quaternion_multiply(roll, pitch))


def angle_between(unit_vector1, unit_vector2):
    """Given two vectors, (possible as homogeneous coordinates) return the angle between them in radians.

    If we're given homogeneous coordinates, we will ignore the last element since it's always 1 or zero.
    """
    if len(unit_vector1) > 3:
        unit_vector1 = unit_vector1[:3]
    if len(unit_vector2) > 3:
        unit_vector2 = unit_vector2[:3]

    # make them unit vectors
    v1 = unit_vector(unit_vector1)
    v2 = unit_vector(unit_vector2)

    return math.acos(np.dot(v1, v2))

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    magnitude = np.linalg.norm(vector)
    if magnitude == 0:
        return vector
    scalar = 1 / magnitude
    return vector * scalar