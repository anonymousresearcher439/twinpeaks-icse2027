#!/usr/bin/env python3

import unittest

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import convert_Lla_to_LlaDict, BriarLla
from dr_onboard_autonomy.states.BetterPath import SetpointAnimator, magnitude

def make_BriarLla(lat, lon, alt, is_amsl=True) -> BriarLla:
    lladict = convert_Lla_to_LlaDict(Lla(lat, lon, alt))
    return BriarLla(lladict, is_amsl=is_amsl)

def lla_to_BriarLla(lla: Lla, is_amsl: bool) -> BriarLla:
    return make_BriarLla(lla.lat, lla.lon, lla.alt, is_amsl=is_amsl)

class TestBetterPath(unittest.TestCase):
    def test_magnitude(self):
        a = Lla(41.70573086523541, -86.24421841999177, 240.18)
        b = a.move_ned(3, 4, 0)

        actual_distance = magnitude(a.distance_ned(b))

        expected_distance = 5
        expected_distance2 = a.distance(b)

        self.assertAlmostEqual(actual_distance, expected_distance2)
        self.assertAlmostEqual(actual_distance, expected_distance)
    
    def test_find_position_setpoint(self):
        start = make_BriarLla(41.70573086523541, -86.24421841999177, 240.0)

        end = start.ellipsoid.lla.move_ned(30, 0, -40)
        end = lla_to_BriarLla(end, is_amsl=False)

        animator = SetpointAnimator(start, end, 1.0)
        
        actual = animator.find_position_setpoint(5.0)
        actual_lla = make_BriarLla(*actual)

        excepted = start.ellipsoid.lla.move_ned(3, 0, -4)
        excepted = lla_to_BriarLla(excepted, is_amsl=False)
        actual_distance = excepted.amsl.lla.distance(actual_lla.amsl.lla)

        ned_actual = start.ellipsoid.lla.distance_ned(actual_lla.ellipsoid.lla)
        ned_expected = (3, 0, -4)
        for actual_component, expected_component in zip(ned_actual, ned_expected):
            self.assertAlmostEqual(actual_component, expected_component)

        # 2 places means centimeter accuracy
        self.assertAlmostEqual(actual_distance, 0.0, places=2)
    
    def test_find_position_snaps_to_end_position(self):
        start = make_BriarLla(41.70573086523541, -86.24421841999177, 240.0)
        end = start.ellipsoid.lla.move_ned(30, 0, -40)
        end = lla_to_BriarLla(end, is_amsl=False)

        animator = SetpointAnimator(start, end, 1.0)
        

        def check_if_at_end_pos(actual_lla_tup):
            for actual_lla, expected_lla in zip(actual_lla_tup, end.amsl.tup):
                self.assertAlmostEqual(actual_lla, expected_lla)
        
        # after 50 seconds, the animator should return the end pos
        actual_50 = animator.find_position_setpoint(50)
        actual_51 = animator.find_position_setpoint(51)
        actual_9000 = animator.find_position_setpoint(9000)

        for pos in [actual_50, actual_51, actual_9000]:
            check_if_at_end_pos(pos)
        


