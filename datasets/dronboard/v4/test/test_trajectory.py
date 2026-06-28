import math
import unittest
from unittest.mock import call, Mock, NonCallableMagicMock, patch

from dr_onboard_autonomy.states.components.trajectory import (
    _find_angular_distance,
    _find_rotation_axis,
    _normalize_angle,
)


class TestTrajectory(unittest.TestCase):
    def test_normalize_angle(self):
        def test_values(value, expected):
            value = math.radians(value)
            expected = math.radians(expected)
            self.assertAlmostEqual(_normalize_angle(value), expected)
        
        # all values are in degrees
        # this was done to make the test more readable
        test_values(0, 0)
        test_values(360, 0)
        test_values(720, 0)
        test_values(-360, 0)
        test_values(-720, 0)
        test_values(90, 90)
        test_values(450, 90)
        test_values(-90, -90)
        test_values(-450, -90)
        test_values(180, 180)
        test_values(540, 180)
        test_values(-180, 180)
        test_values(-540, 180)
        test_values(270, -90)
        test_values(630, -90)
        test_values(-270, 90)
        test_values(-630, 90)