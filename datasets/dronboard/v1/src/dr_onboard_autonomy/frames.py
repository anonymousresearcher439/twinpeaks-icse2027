"""
Data structures to keep track of positions and attitudes over time 
"""

import numpy

from typing import Tuple, Callable

import rospy

from .interpolation import interpolate_pos, interpolate_attitude


Vec3 = Tuple[float, float, float]


Quaternion = Tuple[float, float, float, float]


Record = Tuple[float, Tuple]


InterpolationFunction = Callable[[float, float, Tuple, float, Tuple], Tuple]
# This function will estimate a new data point at time t, given two data points that come before and after.
# 
# For position data, this function will be a linear interpolation between the two 3-D points (i.e. the LERP method)
# For attitude data, this function will be a spherical linear interpolation between the two quaternions (i.e. the SLERP method)
# Returns:
#   Tuple - the estimated data at time t
# 
# Args:
#   t: float - the time we want estimated data for (this is a time.time() value)
#   t0: float - the time when we observed data0 (this is a time.time() value)
#   data0: Tuple - the data we observed at time t0.
#   t1: float - the time when we observed data1 (this is a time.time() value)
#   data1: Tuple the data we observed at time t1


class TimeSeriesData:
    def __init__(self, record_size: int, capacity: int, interpolater: InterpolationFunction):
        self._data = numpy.zeros([capacity, record_size + 1], dtype=numpy.float64)

        self._capacity: int = capacity
        self._next: int = 0
        self._interplater: InterpolationFunction = interpolater
    
    def add(self, t: float, data: Tuple):
        if len(self) > 0:
            t_previous, _ = self[self._next - 1]
            # assert t > t_previous
            if t < t_previous:
                rospy.logwarn("CANNOT ADD TimeSeriesData because the new record must be newer than the old record")
                return
        insertion_index = self._next % self._capacity
        self._data[insertion_index] = t, *data
        self._next += 1
    
    def lookup(self, t: float) -> Tuple:
        r_before, r_after = self._find_nearest(t)
        t0, data0 = r_before
        t1, data1 = r_after
        return self._interplater(t, t0, data0, t1, data1)
    
    def __getitem__(self, index: int)-> Tuple[float, Tuple]:
        real_index = (self._next + index) % len(self)
        data_record = self._data[real_index]
        t = data_record[0]
        data = tuple(data_record[1:])
        return t, data
    
    def __len__(self):
        if self._next < self._capacity:
            return self._next
        return self._capacity
    
    def last(self) -> Tuple[float, Tuple]:
        n = len(self)
        return self[n - 1]
    
    def contains(self, t: float) -> bool:
        if len(self) < 2:
            return False

        # make sure t is newer than our oldest entry
        t0, _ = self[0]
        if t < t0:
            return False
        
        # make sure t is older than our newest entry
        t1, _ = self[len(self) - 1]
        if t1 < t:
            return False
        
        return True
    
    def _find_nearest(self, t: float) -> Tuple[Record, Record]:
        """given a float (representing seconds since epoch) return the records just before and just after

        raise LookupError if the data is missing.
        """
        if not self.contains(t):
            raise LookupError()
        
        # in case we made it this far, then we know we have some data available
        left = 0
        right = len(self)
        while right - left > 1:
            mid = (left + right) // 2
            t_mid, _ = self[mid]
            if t < t_mid:
                right = mid
            else:
                left = mid
        # left should be our nearest entry
        t_nearest, _ = self[left]
        if t < t_nearest:
            right_result = left
            left_result = left - 1
        else:
            left_result = left
            right_result = left + 1

        return self[left_result], self[right_result]


def place_holder_func(t, t0, tup0, t1, tup1) -> Tuple[Record, Record]:
    return tup0


class AttitudeData(TimeSeriesData):
    """ Keep track of attitude data over time. Lets you lookup an attitude by
    interpolating between data points.

    quaternions are interpreted as (X, Y, Z, W) 
    """
    def __init__(self, capacity:int=1024):
        super().__init__(record_size=4, capacity=capacity, interpolater=interpolate_attitude)


class PositionData(TimeSeriesData):
    def __init__(self, capacity:int=1024):
        super().__init__(record_size=3, capacity=capacity, interpolater=interpolate_pos)
