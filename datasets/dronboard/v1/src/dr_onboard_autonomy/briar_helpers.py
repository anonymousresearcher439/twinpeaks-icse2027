from typing import Tuple, TypedDict
from sensor_msgs.msg import NavSatFix
from droneresponse_mathtools import Lla, geoid_height


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
    class PosData:
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
