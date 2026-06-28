from dataclasses import dataclass
import enum
from typing import NamedTuple, NewType, Optional

from droneresponse_mathtools import geoid_height


class ALTITUDE_REFERENCE(enum.Enum):
    """Defines the various altitude reference surfaces
    """
    ELLIPSOID_WGS84 = enum.auto()
    AMSL = enum.auto()
    """Above Mean Sea Level"""


EnuYaw = NewType('EnuYaw', float)
"""Specifies a yaw with respect to the local earth east, north, up frame with units in radians.
A value of zero points east. The angle is specified with the right hand rule around the up axis.
"""


@dataclass
class LlaPosition:
    """Specifies a global latitude, longitude, and altitude position

    Parameters
    ----------
    latitude:
        degrees originating at the equator

    longitude:
        degrees originating at the prime meridian 

    altitude:
        meters above the reference ellipsoid or mean sea level depending on the altitude_ref 
        attribute
    
    """
    latitude: float
    longitude: float
    altitude: float
    altitude_ref: ALTITUDE_REFERENCE


    def to_amsl(self):
        """Updates the altitude to an AMSL reference
        """
        if self.altitude_ref != ALTITUDE_REFERENCE.AMSL:
            self.altitude = self.altitude - geoid_height(self.latitude, self.longitude)
            self.altitude_ref = ALTITUDE_REFERENCE.AMSL


    def to_wgs84_ellipsoid(self):
        """Updates the altitude to an AMSL reference
        """
        if self.altitude_ref != ALTITUDE_REFERENCE.ELLIPSOID_WGS84:
            self.altitude = self.altitude + geoid_height(self.latitude, self.longitude)
            self.altitude_ref = ALTITUDE_REFERENCE.ELLIPSOID_WGS84


class NedAcceleration(NamedTuple):
    """Specifies an acceleration with respect to the local north, east, down frame with units in 
    meters squared per second
    """
    north: float
    east: float
    down: float


class NedPosition(NamedTuple):
    """Specifies a position with respect to the local north, east, down frame with units in meters
    """
    north: float
    east: float
    down: float


class NedVelocity(NamedTuple):
    """Specifies a velocity with respect to the local north, east, down frame with units in 
    meters per second
    """
    north: float
    east: float
    down: float


class Quaternion(NamedTuple):
    """Specifies an orientation relative to a known reference coordinate frame
    """
    x: float
    """influences the angle between axis of rotation and the reference x-axis"""
    y: float
    """influences the angle between axis of rotation and the reference y-axis"""
    z: float
    """influences the angle between axis of rotation and the reference z-axis"""
    w: float
    """cosine of the half-angle of rotation about the axis of rotation"""


class IMU(NamedTuple):
    """Groups together data that is typically provided by and IMU sensor
    """
    acceleration_linear: Optional[NedAcceleration]
    attitude: Optional[Quaternion]