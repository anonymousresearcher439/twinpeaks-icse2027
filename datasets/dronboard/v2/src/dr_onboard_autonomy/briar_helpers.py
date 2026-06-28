from typing import Tuple, TypedDict
from sensor_msgs.msg import NavSatFix

from droneresponse_mathtools import Lla, geoid_height

import numpy as np

class LlaDict(TypedDict):
    latitude: float
    longitude: float
    altitude: float


class QuaternionDict(TypedDict):
    x: float
    y: float
    z: float
    w: float


class EulerAnglesDict(TypedDict):
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def convert_LlaDict_to_tuple(lla: LlaDict) -> Tuple[float, float, float]:
    lat = float(lla['latitude'])
    lon = float(lla['longitude'])
    alt = float(lla['altitude'])
    return float(lat), float(lon), float(alt)


def convert_LlaDict_to_Lla(input: LlaDict) -> Lla:
    lat, lon, alt = convert_LlaDict_to_tuple(input)
    return Lla(lat, lon, alt)


def convert_tuple_to_LlaDict(input: Tuple[float, float, float]) -> LlaDict:
    return {
        "latitude": float(input[0]),
        "longitude": float(input[1]),
        "altitude": float(input[2]),
    }


def convert_Lla_to_LlaDict(input: Lla) -> LlaDict:
    input_tup = convert_Lla_to_tuple(input)
    return convert_tuple_to_LlaDict(input_tup)


def convert_tuple_to_Lla(input: Tuple[float, float, float]) -> Lla:
    lat, lon, alt = input
    return Lla(lat, lon, alt)


def convert_Lla_to_tuple(input: Lla) -> Tuple[float, float, float]:
    return float(input.lat), float(input.lon), float(input.alt)


def amsl_to_ellipsoid(amsl_lla: Tuple[float, float, float]) -> Tuple[float, float, float]:
    lat, lon, alt = amsl_lla
    ellipsoid_alt = alt + geoid_height(lat, lon)
    return float(lat), float(lon), float(ellipsoid_alt)


def ellipsoid_to_amsl(ellipsoid_lla: Tuple[float, float, float]) -> Tuple[float, float, float]:
    lat, lon, alt = ellipsoid_lla
    amsl = alt - geoid_height(lat, lon)
    return float(lat), float(lon), float(amsl)


class BriarLla:
    """This holds position data and helps you access coordinates with its
    altitude specified above mean sea level or above the WGS84 ellipsoid.
    
    It also lets you access this data as as an Lla, a Tuple, or LlaDict.
    """
    @staticmethod
    def from_lla(lla: Lla,  is_amsl:bool=True) -> 'BriarLla':
        """Given an Lla, create a BriarLla instance
        """
        d = convert_Lla_to_LlaDict(lla)
        return  BriarLla(pos=d, is_amsl=is_amsl)

    @staticmethod
    def from_tup(tup: Tuple[float, float, float],  is_amsl:bool=True) -> 'BriarLla':
        """Given an Tuple[float, float, float], create a BriarLla instance
        The elements of the tuple are interpreted as (latitude, longitude, altitude) in that order
        """
        d = convert_tuple_to_LlaDict(tup)
        return  BriarLla(pos=d, is_amsl=is_amsl)

    @staticmethod
    def from_dict(pos: LlaDict, is_amsl=True) -> 'BriarLla':
        """Given an LlaDict, create a BriarLla instance
        """
        # Added for consistency with other static factory methods.
        return BriarLla(pos=pos, is_amsl=is_amsl)

    @staticmethod
    def from_ros(pos: NavSatFix) -> 'BriarLla':
        """Given a position message from ROS, create a BriarLla instance
        NavSatFix objects always have their altitude specified above the ellipsoid
        """
        return briarlla_from_ros_position(pos)

    @staticmethod
    def from_args(latitude: float, longitude: float, altitude: float, is_amsl: bool=True):
        """Given latitude, longitude, and altitude arguments create a BriarLla instance

        Altitude is given in meters.
        """
        d = LlaDict(latitude=latitude, longitude=longitude, altitude=altitude)
        return  BriarLla(pos=d, is_amsl=is_amsl)

    class PosData:
        """Internal data type for holding Lla, tup, and dict
        """
        def __init__(self, lla: Lla):
            self.lla = lla
            self.tup = float(lla.lat), float(lla.lon), float(lla.alt)
            self.dict = convert_tuple_to_LlaDict(self.tup)

    def __init__(self, pos: LlaDict, is_amsl=True):
        pos_tup = convert_LlaDict_to_tuple(pos)
        if not is_amsl:
            pos_tup = ellipsoid_to_amsl(pos_tup)

        amsl_tup = pos_tup
        amsl_lla = convert_tuple_to_Lla(amsl_tup)
        self.amsl = BriarLla.PosData(amsl_lla)

        ellipsoidal_tup = amsl_to_ellipsoid(pos_tup)
        ellipsoidal_lla = convert_tuple_to_Lla(ellipsoidal_tup)
        self.ellipsoid = BriarLla.PosData(ellipsoidal_lla)


def briarlla_from_ros_position(position: NavSatFix) -> BriarLla:
    tup = position.latitude, position.longitude, position.altitude
    return BriarLla(convert_tuple_to_LlaDict(tup), is_amsl=False)


def briarlla_from_lat_lon_alt(lat: float, lon: float, alt: float, is_amsl: bool) -> BriarLla:
    return BriarLla(convert_tuple_to_LlaDict((lat, lon, alt)), is_amsl=is_amsl)


def find_circle_start(
        current_pos: Lla,
        target_pos: Lla,
        target_radius: float,
        target_circle_height: float
    ) -> BriarLla: 
    """Finds circle start location.

    Start at the point on the circle that is nearest to the drone's current position.
    The center of the circle is given by target_pos.
    The circle extends horizontially around the target by the target_radius argument.
    The height specifies the height of the circle above the target.

    This function helps as reach the edge of the circle without flying over the
    target position. This is important in case we're circling a person.

    Args:
        current_pos:
            expects altitude above ellipsoid
        target_pos:
            expects altitude above ellipsoid
        target_radius:
            in meters
        target_circle_height:
            meters above the target
    """
    target2drone_ned = target_pos.distance_ned(current_pos)
    target2drone_ned[2] = 0.0
    magnitude = np.linalg.norm(target2drone_ned)
    if magnitude == 0.0:
        # In this edge case start the circle east of target
        start_lla = target_pos.move_ned(0.0, target_radius, -target_circle_height)
        return BriarLla.from_lla(start_lla, is_amsl=False)

    unit_target2drone = target2drone_ned / magnitude

    h_displacement_ned = unit_target2drone * target_radius
    starting_ground_pos = target_pos.move_ned(*h_displacement_ned)
    starting_pos = starting_ground_pos.move_ned(0, 0, -target_circle_height)
    return BriarLla.from_lla(starting_pos, is_amsl=False)
