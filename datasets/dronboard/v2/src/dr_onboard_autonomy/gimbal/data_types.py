from dataclasses import dataclass
from typing import Tuple

from droneresponse_mathtools import Lla

import rospy

@dataclass(order=True, frozen=True)
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0
    
    def astuple(self) -> Tuple[float, float, float, float]:
        return self.x, self.y, self.z, self.w


@dataclass(order=True, frozen=True)
class DroneData:
    position: Lla = Lla(0.0, 0.0, 0.0)
    attitude: Quaternion = Quaternion()

    def update(self, position:Lla=None, attitude:Quaternion=None):
        if position is None:
            position = self.position
        if attitude is None:
            attitude = self.attitude
        return DroneData(position, attitude)


class AbstractGimbalCalculator:
    @staticmethod
    def find_track_position_attitude(drone_data: DroneData, stare_position: Lla) -> Tuple[float, float, float, float]:
        pass

    @staticmethod
    def find_aircraft_yaw(drone_data: DroneData, stare_position: Lla) -> float:
        return None

    @staticmethod
    def limit(value: float, lower_limit: float, upper_limit: float):
        """The gimbal has limits to the maximum angle it supports.
        This helper function will the cap value at the upper or lower limit. 
        """
        new_value = max(value, lower_limit)
        new_value = min(new_value, upper_limit)
        if new_value != value:
            rospy.logwarn(f"Clampping gimbal angle to {new_value}. The original {value} was outside the range {lower_limit} and {upper_limit}")
        return new_value