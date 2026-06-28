from dataclasses import dataclass, field
import enum
import time
from typing import Any, NamedTuple, NewType, Optional, Protocol, Union

from droneresponse_mathtools import geoid_height


class DATUM_REFERENCE(enum.Enum):
    """Defines the various altitude reference surfaces
    """
    ELLIPSOID_WGS84 = enum.auto()
    AMSL = enum.auto()
    """Above Mean Sea Level"""


EnuYaw = NewType('EnuYaw', float)
"""Specifies a yaw with respect to the local earth east, north, up frame with units in radians.
A value of zero points east. The angle is specified with the right hand rule around the up axis.
"""


RelativeAltitude = NewType('RelativeAltitude', float)
"""Specifies an altitude relative to the drone's takeoff position with units in meters"""


@dataclass(frozen=True)
class LlaPosition:
    """Specifies a global latitude, longitude, and altitude position

    Parameters
    ----------
    latitude:
        degrees originating at the equator

    longitude:
        degrees originating at the prime meridian 

    altitude:
        meters above the reference ellipsoid or mean sea level depending on the datum_ref 
        attribute
    
    datum_ref:
        the reference surface for the altitude value
    
    """
    latitude: float
    longitude: float
    altitude: float
    datum_ref: DATUM_REFERENCE

    def to_amsl(self):
        """Updates the altitude to an AMSL reference
        """
        if self.datum_ref != DATUM_REFERENCE.AMSL:
            geo_offset = geoid_height(self.latitude, self.longitude)
            assert geo_offset is not None, "Geoid height could not be determined"
            altitude = self.altitude - geo_offset
            datum_ref = DATUM_REFERENCE.AMSL
            return LlaPosition(latitude=self.latitude, longitude=self.longitude, altitude=altitude, datum_ref=datum_ref)
        else:
            return self


    def to_wgs84_ellipsoid(self):
        """Updates the altitude to a WGS84 ellipsoid reference
        """
        if self.datum_ref != DATUM_REFERENCE.ELLIPSOID_WGS84:
            geo_offset = geoid_height(self.latitude, self.longitude)
            assert geo_offset is not None, "Geoid height could not be determined"
            altitude = self.altitude + geo_offset
            altitude_ref = DATUM_REFERENCE.ELLIPSOID_WGS84
            return LlaPosition(latitude=self.latitude, longitude=self.longitude, altitude=altitude, datum_ref=altitude_ref)
        else:
            return self
    
    @staticmethod
    def from_lla(lla_like_object, is_amsl: bool) -> 'LlaPosition':
        """Creates a LlaPosition from an object that has attributes or properties for latitude, longitude, and altitude.  
        """
        datum = DATUM_REFERENCE.ELLIPSOID_WGS84
        if is_amsl:
            datum = DATUM_REFERENCE.AMSL
        
        return LlaPosition(
            latitude=lla_like_object.latitude,
            longitude=lla_like_object.longitude,
            altitude=lla_like_object.altitude,
            datum_ref=datum
        )


# Protocol for objects with latitude, longitude, and altitude attributes
class LlaLike(Protocol):
    latitude: float
    longitude: float
    altitude: float

@dataclass(frozen=True)
class TimeStampedLlaPosition(LlaPosition):
    
    time: float

    def to_amsl(self):
        result = super().to_amsl()
        return TimeStampedLlaPosition(
            time=self.time,
            latitude=result.latitude,
            longitude=result.longitude,
            altitude=result.altitude,
            datum_ref=result.datum_ref
        )

    def to_wgs84_ellipsoid(self):
        result = super().to_wgs84_ellipsoid()
        return TimeStampedLlaPosition(
            time=self.time,
            latitude=result.latitude,
            longitude=result.longitude,
            altitude=result.altitude,
            datum_ref=result.datum_ref
        )


@dataclass(frozen=True)
class GpsPositionMessage(TimeStampedLlaPosition):
    is_fix: bool


class NedVector(NamedTuple):
    """Specifies a vector with respect to the local north, east, down frame with units in meters
    """
    north: float
    east: float
    down: float


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


@dataclass(frozen=True)
class Quaternion:
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

    def __getitem__(self, index):
        return (self.x, self.y, self.z, self.w)[index]

    def to_dict(self) -> dict:
        """Return a JSON-serializable representation of this quaternion.

        Returns
        -------
        dict
            {"x": float, "y": float, "z": float, "w": float}
        """
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }

@dataclass(frozen=True)
class TimeStampedQuaternion(Quaternion):
    time: float


# TODO should the IMU message have a TimeStampedQuaternion instead of a Quaternion?
class IMU(NamedTuple):
    """Groups together data that is typically provided by and IMU sensor
    """
    time: float
    acceleration_linear: Optional[NedAcceleration]
    attitude: Optional[Quaternion]


class TimeSource(enum.Enum):
    """Specifies the source of a time reference
    """
    UNKNOWN = enum.auto()
    """The source of the time reference is unknown"""
    FCU = enum.auto()
    """The source of the time reference is the flight control unit"""


@dataclass(frozen=True)
class TimeRef:
    """Specifies a time reference
    """
    nanoseconds: int
    """nanoseconds since the epoch as provided by the source"""
    source: TimeSource
    """the source of the time reference"""
    receive_time: int = field(default_factory=time.monotonic_ns)
    """Records the moment when this TimeRef object was initalized"""

    def adjusted_timestamp_ns(self) -> int:
        """Calculates an adjusted timestamp based on the receive time.

        This method is used to account for processing delays when setting the clock
        using GPS time.
        
        There is a non-trivial time lag between when we receive the timestamp and
        when we try to set the system clock. This method helps to
        account for that time lag.
         
        The adjusted timestamp is calculated by adding an offset to the original
        timestamp. The offset is the difference between the current time and the
        time when the TimeRef object was created (i.e., when the GPS time was received).

        This adjustment helps us set the clock more accurately by compensating
        for the time that elapses between receiving the GPS time and actually
        setting the clock.

        Returns:
            int: The adjusted timestamp in nanoseconds.
        """
        # TODO account for additional time delays like transmission time
        offset_ns = time.monotonic_ns() - self.receive_time
        return self.nanoseconds + offset_ns
    