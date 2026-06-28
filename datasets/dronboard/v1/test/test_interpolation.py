import unittest
from typing import Tuple

Vec3 = Tuple[float, float, float]

class TestInterpolation(unittest.TestCase):
    def test_interpolate_pos(self):
        from droneresponse_mathtools import Lla
        from dr_onboard_autonomy.interpolation import interpolate_pos
        # from src.dr_onboard_autonomy.mavros_layer import Vec3

        pos_0: Vec3 = Lla(39.74827, -105.04570, 5312.26).to_pvector()
        pos_1: Vec3 = Lla(39.74772, -105.04540, 5295.73).to_pvector()
        t_0 = 1.2345
        t_1 = 1.3376

        t_to_interpolate = 1.2889

        int_pos = interpolate_pos(t_to_interpolate, t_0, pos_0, t_1, pos_1)

        expected_interpolated_pos = (-1275797.354018113,-4746247.057868786,4059901.49265676)

        for calced, expect in zip(int_pos, expected_interpolated_pos):
            self.assertAlmostEqual(calced, expect)


    def test_interpolate_attitude(self):
        from src.dr_onboard_autonomy.interpolation import interpolate_attitude
        from src.dr_onboard_autonomy.gimbal.data_types import Quaternion

        '''
        SLERP calculator used to compute expected outputs from randomly generated quaternions
        https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
        '''
        test_sequence = {
            "t_int = 0.5 with q0 rotated pi/4 and q1 rotated 3pi/4 about x-axis": {
                "q_0": (0.0, 0.38268343236, 0.0, 0.92387953251),
                "q_1": (0.0, 0.92387953251, 0.0, 0.38268343236),
                "t_0": 2.0,
                "t_1": 3.0,
                "t_int": 2.5,
                "q_expected": (0.0, 0.7071067811852485, 0.0, 0.7071067811852485)
            },
            "t_int = 0.34 with randomly generated quaternions": {
                "q_0": (0.23609, 0.4685, 0.655733, 0.54294),
                "q_1": (0.105368, 0.3969, 0.7775, 0.4763),
                "t_0": 0.0,
                "t_1": 1.0,
                "t_int": 0.34,
                "q_expected": (0.1924779, 0.4462034, 0.7004646, 0.5226893)
            }
        }

        for test_desc, test_values in test_sequence.items():
            with self.subTest(test_desc):
                interpolated_q = interpolate_attitude(
                    test_values["t_int"],
                    test_values["t_0"],
                    test_values["q_0"],
                    test_values["t_1"],
                    test_values["q_1"]
                )

                self.assertAlmostEqual(interpolated_q[0], test_values["q_expected"][0])
                self.assertAlmostEqual(interpolated_q[1], test_values["q_expected"][1])
                self.assertAlmostEqual(interpolated_q[2], test_values["q_expected"][2])
                self.assertAlmostEqual(interpolated_q[3], test_values["q_expected"][3])


