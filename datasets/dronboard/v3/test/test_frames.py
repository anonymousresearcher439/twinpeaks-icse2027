import unittest
from unittest.mock import Mock

import numpy

from dr_onboard_autonomy.models.frames import (
    AttitudeData,
    PositionData,
    PositionDataLla,
    TimeSeriesData
)


class MockTime:
    def __init__(self, t0=1670444212.6641, incremental_change=0.1):
        self.t0 = t0
        self.delta = incremental_change
        self.count = 0

    def time(self):
        result_time = self.t0 + self.count * self.delta
        self.count += 1
        return result_time

def mock_interpolater(t0, tup0, t1, tup1):
    return tup0


class TestTimeSeriesData(unittest.TestCase):

    def assert_records_match(self, record_a, record_b):
        t_a, vec_a = record_a
        t_b, vec_b = record_b
        self.assertEqual(t_a, t_b)
        self.assertEqual(vec_a, vec_b)


    def test_position_data_len(self):
        mock_time = MockTime()
        p = TimeSeriesData(3, 30, mock_interpolater)
        for i in range(30):
            assert len(p) == i
            f = float(i)
            vec3 = f, f, f 
            p.add(mock_time.time(), vec3)
            assert len(p) == i + 1
        
        for i in range(30, 60, 1):
            assert len(p) == 30
            f = float(i)
            vec3 = f, f, f 
            p.add(mock_time.time(), vec3)

        assert len(p) == 30

    def test_get_item(self):
        clock = MockTime(t0=0.0, incremental_change=1.0)
        p = TimeSeriesData(record_size=3, capacity=30, interpolater=mock_interpolater)
        for i in range(30):
            f = float(i)
            vec3 = f, f, f 
            p.add(clock.time(), vec3)
        
        t, vec3 = p[0]
        t_expected = 0.0
        vec3_expected = 0.0, 0.0, 0.0
        assert t == t_expected
        assert vec3 == vec3_expected

        for i in range(30):
            t_expected = float(i)
            vec3_expected = float(i), float(i), float(i)

            t, vec3 = p[i]
            assert t == t_expected
            assert vec3 == vec3_expected
        
        for i in range(30, 60):
            f = float(i)
            vec = f, f, f
            t = f
            p.add(t, vec)
        
        for i in range(30):
            f = float(i) + 30.0
            t_expected = f
            vec3_expected = f, f, f

            t, vec3 = p[i]
            assert t == t_expected
            assert vec3 == vec3_expected
        
        tsd = TimeSeriesData(record_size=3, capacity=20, interpolater=mock_interpolater)
        tsd.add(0, (1, 1, 1))
        t, v = tsd[0]
        assert t == 0.0
        assert v == (1.0, 1.0, 1.0)
    
    def test_find_nearest_with_no_elements(self):
        """query for records when there isn't any data  
        """

        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        self.assertRaises(LookupError, tsd._find_nearest, 1.0)
    
    def test_find_nearest_with_1_element(self):
        """Add 1 element and query for data. Make sure the error is raised
        """
        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        tsd.add(1.0, (1, 1, 1))
        self.assertRaises(LookupError, tsd._find_nearest, 1.0)

    def test_find_nearest_with_2_elements_when_query_time_is_out_of_bounds(self):
        """Add 2 elements and query for data that's older than the oldest record.
        Also query for data that is newer than the newest record
        Finaly query for data that is in bounds
        
        """
        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        # oldest data is a time = 3.0
        tsd.add(3.0, (3.0, 3.0, 3.0))
        # newest data is a time = 5.0
        tsd.add(5.0, (5.0, 5.0, 5.0))

        self.assertRaises(LookupError, tsd._find_nearest, 2.0)
        self.assertRaises(LookupError, tsd._find_nearest, 6.0)

        r0, r1 = tsd._find_nearest(4.0)
        t0, v0 = r0
        t1, v1 = r1
        assert t0 == 3.0
        assert t1 == 5.0
        assert v0 == (3.0, 3.0, 3.0)
        assert v1 == (5.0, 5.0, 5.0)
    
    def test_find_nearest_with_two_elements(self):
        """Add two records and query for them with various inputs  
        """

        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        tsd.add(1.0, (1.0, 1.0, 1.0))
        tsd.add(2.0, (2.0, 2.0, 2.0))

        for t in numpy.linspace(1.1, 1.9, 12):
            r0, r1 = tsd._find_nearest(t)
            t0, vec0 = r0
            t1, vec1 = r1
            assert t0 == 1.0
            assert t1 == 2.0

            assert vec0 == (1.0, 1.0, 1.0)
            assert vec1 == (2.0, 2.0, 2.0)
    
    def test_find_nearest_with_three_elements(self):
        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        tsd.add(1.0, (1.0, 1.0, 1.0))
        tsd.add(2.0, (2.0, 2.0, 2.0))
        tsd.add(3.0, (3.0, 3.0, 3.0))

        r_left, r_right = tsd._find_nearest(1.5)
        self.assert_records_match(r_left, tsd[0])
        self.assert_records_match(r_right, tsd[1])

        r_left, r_right = tsd._find_nearest(2.5)
        self.assert_records_match(r_left, tsd[1])
        self.assert_records_match(r_right, tsd[2])

    def test_find_nearest_with_four_elements(self):
        tsd = TimeSeriesData(record_size=3, capacity=100, interpolater=mock_interpolater)
        tsd.add(1.0, (1.0, 1.0, 1.0))
        tsd.add(2.0, (2.0, 2.0, 2.0))
        tsd.add(3.0, (3.0, 3.0, 3.0))
        tsd.add(4.0, (4.0, 4.0, 4.0))


        r_left, r_right = tsd._find_nearest(1.5)
        self.assert_records_match(r_left, tsd[0])
        self.assert_records_match(r_right, tsd[1])

        r_left, r_right = tsd._find_nearest(2.5)
        self.assert_records_match(r_left, tsd[1])
        self.assert_records_match(r_right, tsd[2])

        r_left, r_right = tsd._find_nearest(3.5)
        self.assert_records_match(r_left, tsd[2])
        self.assert_records_match(r_right, tsd[3])

    def test_find_nearest_with_many_elements(self):
        tsd = TimeSeriesData(record_size=2, capacity=99, interpolater=mock_interpolater)
        for i in range(1, 51):
            vec = (i, i)
            tsd.add(float(i), vec)
        
        for i in range(1, 50):
            t = i + 0.5
            left = i - 1
            right = i
            r_left, r_right = tsd._find_nearest(t)
            r_left_expected = tsd[left]
            r_right_expected = tsd[right]
            self.assert_records_match(r_left, r_left_expected)
            self.assert_records_match(r_right, r_right_expected)
    
    def test_find_nearest_with_even_capacity_and_full(self):
        mock_clock = MockTime()
        vec_value = 1.0
        cap = 128
        tsd = TimeSeriesData(record_size=2, capacity=cap, interpolater=mock_interpolater)
        for _ in range(cap * 3 // 2):
            t = mock_clock.time()
            vec = vec_value, vec_value
            tsd.add(t, vec)
            vec_value += 1
        
        
        for i in range(cap - 1):
            t_left, vec_left = tsd[i]
            t = t_left + 0.05

            r_left, r_right = tsd._find_nearest(t)
            expected_left = t_left, vec_left
            expected_right = tsd[i + 1]
            self.assert_records_match(r_left, expected_left)
            self.assert_records_match(r_right, expected_right)

    def test_find_nearest_with_even_capacity_and_full2(self):
        mock_clock = MockTime()
        vec_value = 1.0
        cap = 1024
        tsd = TimeSeriesData(record_size=2, capacity=cap, interpolater=mock_interpolater)
        for _ in range(cap * 3 // 2):
            t = mock_clock.time()
            vec = vec_value, vec_value
            tsd.add(t, vec)
            vec_value += 1
        
        for i in range(cap - 1):
            t_left, vec_left = tsd[i]
            expected_left = t_left, vec_left
            expected_right = tsd[i + 1]
            for t in numpy.linspace(t_left + 0.001, t_left + (0.1 - 0.001), 12):
                r_left, r_right = tsd._find_nearest(t)
                self.assert_records_match(r_left, expected_left)
                self.assert_records_match(r_right, expected_right)

    def test_find_nearest_with_odd_capacity_and_full(self):
        mock_clock = MockTime()
        vec_value = 1.0
        cap = 1024 - 1
        tsd = TimeSeriesData(record_size=2, capacity=cap, interpolater=mock_interpolater)
        for _ in range(cap * 3 // 2):
            t = mock_clock.time()
            vec = vec_value, vec_value
            tsd.add(t, vec)
            vec_value += 1
        
        for i in range(cap - 1):
            t_left, vec_left = tsd[i]
            expected_left = t_left, vec_left
            expected_right = tsd[i + 1]
            for t in numpy.linspace(t_left + 0.001, t_left + (0.1 - 0.001), 12):
                r_left, r_right = tsd._find_nearest(t)
                self.assert_records_match(r_left, expected_left)
                self.assert_records_match(r_right, expected_right)
    
    def test_lerp(self):
        tsd = PositionData()
        t0 = 0
        t1 = 1

        p0 = 0,0,0
        p1 = 1,0,0
        tsd.add(t0, p0)
        tsd.add(t1, p1)
        x, y, z = tsd.lookup(0.5)

        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.0)
    
    def test_slerp(self):
        tsd = AttitudeData()
        t0 = 0
        t1 = 1

        from math import sin, cos, pi

        q0 = (0, 0, 0, 1)
        q1 = (sin(pi/8), 0, 0, cos(pi/8))
        tsd.add(t0, q0)
        tsd.add(t1, q1)
        q_i = tsd.lookup(0.5)
        q_expected = (sin(pi/16), 0, 0, cos(pi/16))

        self.assertAlmostEqual(q_i[0], q_expected[0])
        self.assertAlmostEqual(q_i[1], q_expected[1])
        self.assertAlmostEqual(q_i[2], q_expected[2])
        self.assertAlmostEqual(q_i[3], q_expected[3])


class TestPositionData(unittest.TestCase):
    def test_add_position(self):
        from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition

        position_data = PositionDataLla()
        lla_pos = LlaPosition(
            latitude=39.782,
            longitude=-105.030,
            altitude=6015,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )
        position_data.add_position(1718399626, lla_pos)

        self.assertEqual(
            position_data.last(),
            (1718399626, (lla_pos.latitude, lla_pos.longitude, lla_pos.altitude))
        )


    def test_add_position_amsl(self):
        """LlaPositions should always have a wgs84 ellipsoid altitude reference before
        being added to TimeSeriesData vector"""
        from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition

        position_data = PositionDataLla()
        lla_pos = LlaPosition(
            latitude=39.782,
            longitude=-105.030,
            altitude=2214,
            datum_ref=DATUM_REFERENCE.AMSL
        )
        position_data.add_position(1718399626, lla_pos)

        lla_pos = lla_pos.to_wgs84_ellipsoid()
        self.assertEqual(
            position_data.last(),
            (1718399626, (lla_pos.latitude, lla_pos.longitude, lla_pos.altitude))
        )


    def test_lookup_position(self):
        from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition

        position_data = PositionDataLla()
        lla_pos = LlaPosition(
            latitude=39.782,
            longitude=-105.030,
            altitude=2214,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )
        # lookup is already tested separately, so assume it finds the correct position
        position_data.lookup = Mock(
            return_value=(
                lla_pos.latitude,
                lla_pos.longitude,
                lla_pos.altitude
            ))

        matched_lla_pos = position_data.lookup_position(1718402524)
        self.assertIsInstance(matched_lla_pos, LlaPosition)
        self.assertEqual(
            (
                matched_lla_pos.latitude,
                matched_lla_pos.longitude,
                matched_lla_pos.altitude
            ),
            (
                lla_pos.latitude,
                lla_pos.longitude,
                lla_pos.altitude
            )
        )


    def test_get_position(self):
        from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition

        position_data = PositionDataLla()

        position_data.add_position(1718399626, LlaPosition(
            latitude=39.782,
            longitude=-105.030,
            altitude=6015,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        ))
        expected_latest_pos = LlaPosition(
            latitude=22.423,
            longitude=-106.112,
            altitude=8952,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )
        position_data.add_position(1718399845, expected_latest_pos)

        latest_pos = position_data.get_position()

        self.assertEqual(
            (
                latest_pos.latitude,
                latest_pos.longitude,
                latest_pos.altitude
            ),
            (
                expected_latest_pos.latitude,
                expected_latest_pos.longitude,
                expected_latest_pos.altitude
            )
        )


    def test_get_position_no_positions_available(self):
        position_data = PositionDataLla()

        self.assertIsNone(position_data.get_position())


class TestAttitudeData(unittest.TestCase):
    def test_add_attitude(self):
        import math
        from dr_onboard_autonomy.models import AttitudeData
        from dr_onboard_autonomy.models import Quaternion

        attitude_data = AttitudeData()

        attitude_data.add_attitude(
            time=1718647457,
            attitude=Quaternion(
                x=math.sin((math.pi / 2) / 2), # +90 deg rotation about x
                y=0,
                z=0,
                w=math.cos((math.pi / 2) / 2)
            )
        )

        expected_attitude = attitude_data.last()

        self.assertAlmostEqual(expected_attitude[0], 1718647457)
        self.assertAlmostEqual(expected_attitude[1][0], math.sin((math.pi / 2) / 2))
        self.assertAlmostEqual(expected_attitude[1][1], 0)
        self.assertAlmostEqual(expected_attitude[1][2], 0)
        self.assertAlmostEqual(expected_attitude[1][3], math.cos((math.pi / 2) / 2))


    def test_lookup_attitude(self):
        import math
        from dr_onboard_autonomy.models import AttitudeData

        attitude_data = AttitudeData()
        rotation_angle = (math.pi / 3) / 2

        # lookup is already tested separately, so assume it finds the correct position
        attitude_data.lookup = Mock(
            return_value=(
                math.sin(rotation_angle),
                0,
                0,
                math.cos(rotation_angle)
            ))

        expected_attitude = attitude_data.lookup_attitude(time=235.13)

        self.assertAlmostEqual(expected_attitude.x, math.sin(rotation_angle))
        self.assertAlmostEqual(expected_attitude.y, 0)
        self.assertAlmostEqual(expected_attitude.z, 0)
        self.assertAlmostEqual(expected_attitude.w, math.cos(rotation_angle))


    def test_get_attitude(self):
        import math
        from dr_onboard_autonomy.models import AttitudeData, Quaternion

        attitude_data = AttitudeData()
        rotation_angle = (math.pi / 4) / 2

        attitude_data.add_attitude(
            time=1718647457,
            attitude=Quaternion(
                x=math.sin(rotation_angle), # +90 deg rotation about x
                y=0,
                z=0,
                w=math.cos(rotation_angle)
            )
        )

        expected_attitude = attitude_data.get_attitude()

        self.assertAlmostEqual(expected_attitude.x, math.sin(rotation_angle))
        self.assertAlmostEqual(expected_attitude.y, 0)
        self.assertAlmostEqual(expected_attitude.z, 0)
        self.assertAlmostEqual(expected_attitude.w, math.cos(rotation_angle))