from dataclasses import dataclass
import enum
from typing import NewType

from pymavlink.dialects.v20 import common as mavlink2


class MavType(enum.Enum):
    GENERIC = mavlink2.MAV_TYPE_GENERIC  # 0 - Generic micro air vehicle
    FIXED_WING = mavlink2.MAV_TYPE_FIXED_WING  # 1 - Fixed wing aircraft.
    QUADROTOR = mavlink2.MAV_TYPE_QUADROTOR  # 2 - Quadrotor
    COAXIAL = mavlink2.MAV_TYPE_COAXIAL  # 3 - Coaxial helicopter
    HELICOPTER = mavlink2.MAV_TYPE_HELICOPTER  # 4 - Normal helicopter with tail rotor.
    ANTENNA_TRACKER = mavlink2.MAV_TYPE_ANTENNA_TRACKER  # 5 - Ground installation
    GCS = mavlink2.MAV_TYPE_GCS  # 6 - Operator control unit / ground control station
    AIRSHIP = mavlink2.MAV_TYPE_AIRSHIP  # 7 - Airship, controlled
    FREE_BALLOON = mavlink2.MAV_TYPE_FREE_BALLOON  # 8 - Free balloon, uncontrolled
    ROCKET = mavlink2.MAV_TYPE_ROCKET  # 9 - Rocket
    GROUND_ROVER = mavlink2.MAV_TYPE_GROUND_ROVER  # 10 - Ground rover
    SURFACE_BOAT = mavlink2.MAV_TYPE_SURFACE_BOAT  # 11 - Surface vessel, boat, ship
    SUBMARINE = mavlink2.MAV_TYPE_SUBMARINE  # 12 - Submarine
    HEXAROTOR = mavlink2.MAV_TYPE_HEXAROTOR  # 13 - Hexarotor
    OCTOROTOR = mavlink2.MAV_TYPE_OCTOROTOR  # 14 - Octorotor
    TRICOPTER = mavlink2.MAV_TYPE_TRICOPTER  # 15 - Tricopter
    FLAPPING_WING = mavlink2.MAV_TYPE_FLAPPING_WING  # 16 - Flapping wing
    KITE = mavlink2.MAV_TYPE_KITE  # 17 - Kite
    ONBOARD_CONTROLLER = mavlink2.MAV_TYPE_ONBOARD_CONTROLLER  # 18 - Onboard companion controller
    VTOL_DUOROTOR = mavlink2.MAV_TYPE_VTOL_DUOROTOR  # 19 - Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
    VTOL_QUADROTOR = mavlink2.MAV_TYPE_VTOL_QUADROTOR  # 20 - Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
    VTOL_TILTROTOR = mavlink2.MAV_TYPE_VTOL_TILTROTOR  # 21 - Tiltrotor VTOL
    VTOL_RESERVED2 = mavlink2.MAV_TYPE_VTOL_RESERVED2  # 22 - VTOL reserved 2
    VTOL_RESERVED3 = mavlink2.MAV_TYPE_VTOL_RESERVED3  # 23 - VTOL reserved 3
    VTOL_RESERVED4 = mavlink2.MAV_TYPE_VTOL_RESERVED4  # 24 - VTOL reserved 4
    VTOL_RESERVED5 = mavlink2.MAV_TYPE_VTOL_RESERVED5  # 25 - VTOL reserved 5
    GIMBAL = mavlink2.MAV_TYPE_GIMBAL  # 26 - Gimbal
    ADSB = mavlink2.MAV_TYPE_ADSB  # 27 - ADSB system
    PARAFOIL = mavlink2.MAV_TYPE_PARAFOIL  # 28 - Steerable, nonrigid airfoil
    DODECAROTOR = mavlink2.MAV_TYPE_DODECAROTOR  # 29 - Dodecarotor
    CAMERA = mavlink2.MAV_TYPE_CAMERA  # 30 - Camera
    CHARGING_STATION = mavlink2.MAV_TYPE_CHARGING_STATION  # 31 - Charging station
    FLARM = mavlink2.MAV_TYPE_FLARM  # 32 - FLARM collision avoidance system
    SERVO = mavlink2.MAV_TYPE_SERVO  # 33 - Servo
    ODID = mavlink2.MAV_TYPE_ODID  # 34 - Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.
    DECAROTOR = mavlink2.MAV_TYPE_DECAROTOR  # 35 - Decarotor
    BATTERY = mavlink2.MAV_TYPE_BATTERY  # 36 - Battery
    PARACHUTE = mavlink2.MAV_TYPE_PARACHUTE  # 37 - Parachute


class MavState(enum.Enum):
    UNINIT = mavlink2.MAV_STATE_UNINIT  # 0 - Uninitialized system, state is unknown.
    BOOT = mavlink2.MAV_STATE_BOOT  # 1 - System is booting up.
    CALIBRATING = mavlink2.MAV_STATE_CALIBRATING  # 2 - System is calibrating and not flight-ready.
    STANDBY = mavlink2.MAV_STATE_STANDBY  # 3 - System is grounded and on standby. It can be launched any time.
    ACTIVE = mavlink2.MAV_STATE_ACTIVE  # 4 - System is active and might be already airborne. Motors are engaged.
    CRITICAL = mavlink2.MAV_STATE_CRITICAL  # 5 - System is in a non-normal flight mode. It can however still navigate.
    EMERGENCY = mavlink2.MAV_STATE_EMERGENCY  # 6 - System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
    POWEROFF = mavlink2.MAV_STATE_POWEROFF  # 7 - System just initialized its power-down sequence, will shut down now.
    FLIGHT_TERMINATION = mavlink2.MAV_STATE_FLIGHT_TERMINATION  # 8 - System is terminating itself.


SystemId = NewType('SystemId', int)
ComponentId = NewType('ComponentId', int)


@dataclass(order=True, frozen=True)
class MavlinkNode:
    system_id: SystemId
    component_id: ComponentId