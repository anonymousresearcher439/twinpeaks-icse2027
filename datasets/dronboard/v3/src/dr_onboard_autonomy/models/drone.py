from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Optional, Tuple, Union
import enum

from .frames import AttitudeData, PositionDataLla
from .gimbal import Gimbal
from .kinematics import (
    EnuYaw,
    LlaPosition,
    NedAcceleration,
    NedPosition,
    NedVelocity,
    Quaternion,
    RelativeAltitude,
    TimeStampedLlaPosition,
)  


@dataclass(frozen=True)
class Battery:
    """
    A class to represent the battery state of a drone.

    Attributes
    ----------
    voltage : float
        The voltage of the battery in Volts.
    current : float
        How much current the drone is using in Amperes.
    level : float
        How much energy is left in the battery. This is a value from 0 to 1.
    """

    voltage: float
    """The voltage of the battery in Volts."""
    current: float
    """How much current the drone is using in Amperes."""
    level: float
    """How much energy is left in the battery. This is a value from 0 to 1."""


class FCUCopterMode(enum.Enum):
    ALT_HOLD = enum.auto()
    """Holds the altitude only. Drone will drift horizontally wiht outside forces"""
    LAND = enum.auto()
    """Flight controller alone immediately manages landing the vehicle"""
    LOITER = enum.auto()
    """Flight controller holds current altitude and position with no response to stick movements"""
    MANUAL_OTHER = enum.auto()
    """Pilot has full control over the drone using manual mode or another mode not listed here"""
    MISSION = enum.auto()
    """Flight controller executed flight path based on predefined mission"""
    OFFBOARD = enum.auto()
    """Obeys setpoint messages sent from a companion controller"""
    POS_HOLD = enum.auto()
    """Holds the 3D position of the drone even against outside forces"""
    RTL = enum.auto()
    """Flight controller executes its return to land procedure"""
    STABILIZE = enum.auto()
    """Vehicle horizontal posture will be held flat when sticks released"""
    TAKEOFF = enum.auto()
    """Flight controller alone manages lifting the vehicle into the air"""
    UNKNOWN = enum.auto()
    """Mode is unknown"""


FCU_MODES_HUMAN_CONTROL = set((
    FCUCopterMode.ALT_HOLD.value,
    FCUCopterMode.LAND.value,
    FCUCopterMode.MANUAL_OTHER.value,
    FCUCopterMode.POS_HOLD.value,
    FCUCopterMode.RTL.value,
    FCUCopterMode.STABILIZE.value,
))


class FCUStatus(enum.Enum):
    UNKNOWN = enum.auto()
    BOOT = enum.auto()
    CALIBRATE = enum.auto()
    STANDBY = enum.auto()
    """System is grounded, but ready to launch"""
    ACTIVE = enum.auto()
    """The motors are engaged and the system may be airborne"""
    CRITICAL = enum.auto()
    """Non-normal, but can still navigate"""
    EMERGENCY = enum.auto()
    """Lost control over parts of or the entire airframe"""
    POWER_OFF = enum.auto()
    """Power down sequence initiated"""
    TERMINATE = enum.auto()


class FCULandedStatus(enum.Enum):
    UNKNOWN = enum.auto()
    ON_GROUND = enum.auto()
    IN_AIR = enum.auto()


@dataclass(frozen=True)
class FCUState:
    """Holds data related to the flight controller state
    """
    armed: bool
    connected: bool
    mode: FCUCopterMode
    status: FCUStatus
    landed_status: FCULandedStatus = FCULandedStatus.UNKNOWN


class HeartbeatStatus(enum.Enum):
    """Defines drone action based on reception of a ground control heartbeat"""
    HOVER = enum.auto()
    """The drone should hover in place due to a lost ground control heartbeat"""
    RTL = enum.auto()
    """The drone should return to land due to a lost ground control heartbeat"""
    CONTINUE = enum.auto()
    """The drone has an active heartbeat from ground control and should continue normal operation"""


@dataclass
class SetpointLatest():
    lla: Optional[LlaPosition] = None
    ned_position: Optional[NedPosition] = None
    ned_velocity: Optional[NedVelocity] = None
    yaw: Optional[EnuYaw] = None


@dataclass(frozen=True)
class CopterParameters:

    takeoff_altitude: float = field(metadata={"help": "The default Takeoff altitude provided in meters. We load this value from the flight controller's parameter list, if possible. If not, we use a sensible default value."})
    """The default Takeoff altitude provided in meters. We load this value from the flight controller's parameter list, if possible. If not, we use a sensible default value."""

    horizontal_acceleration_limit: float = field(metadata={"help": "The maximum horizontal acceleration in m/s^2"})
    """The maximum horizontal acceleration in m/s^2"""

    upward_acceleration_limit: float = field(metadata={"help": "The maximum acceleration upwards in m/s^2"})
    """The maximum acceleration upwards in m/s^2"""

    downward_acceleration_limit: float = field(metadata={"help": "The maximum acceleration downward in m/s^2"})
    """The maximum acceleration downward in m/s^2"""

    jerk_limit: float = field(metadata={"help": "The maximum jerk in m/s^3"})
    """The maximum jerk in m/s^3"""
    
    maximum_yaw_rate: float = field(metadata={"help": "The maximum yaw rate in radians per second"})
    """The maximum yaw rate in radians per second"""


class CopterDrone(ABC):
    def __init__(self, gimbal: Optional[Gimbal], uav_name: str):
        self.uav_name = uav_name
        self.gimbal = gimbal

        self.data = Data()
        self.data.uav_id = uav_name
        self._fcu_mode: Optional[FCUCopterMode] = None


    @abstractmethod
    def arm(self) -> bool:
        """Arms the flight control unit. Returns True if successful.

        Returns
        -------
        bool
            True if successful
        """


    # TODO: determine if there is a way to keep the need for FCU specific knowledge contained to the 
    # drone object 
    @abstractmethod
    def get_fcu_parameter(self, name: str) -> Optional[Union[float, int]]:
        """Returns the value of a flight control unit parameter

        Parameters
        ----------
        name
            The parameter name is flight control unit specific

        Returns
        -------
        None if parameter retrieval is unsuccessful
        """


    @abstractmethod
    def set_fcu_parameter(self, name: str, value: Union[float, int]) -> bool:
        """Sets the specified flight control unit parameter with the specified value.
        Returns True if successful.
        
        Parameters
        ----------
        name
            The parameter name is flight control unit specific


        Returns
        -------
        success_flag
            True if successful
        """


    @abstractmethod
    def start(self):
        """Initiates communication with the flight control unit
        """


    def send_lla_setpoint(
        self,
        lla: LlaPosition,
        yaw: EnuYaw=EnuYaw(0.0)):
        """Sends a desired global position and yaw to the flight controller
        """
        self.data.setpoint_latest.lla = lla
        self.data.setpoint_latest.yaw = yaw

        self._send_lla_setpoint(
            lla=lla,
            yaw=yaw
        )


    def send_ned_setpoint(
        self,
        ned_position: NedPosition,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        """Sends a desired local north, east, down position relative to the drone's takeoff position
        """
        self.data.setpoint_latest.ned_position = ned_position
        self.data.setpoint_latest.yaw = yaw

        self._send_ned_setpoint(
            ned_position=ned_position,
            yaw=yaw
        )


    def send_ned_velocity_setpoint(
        self,
        ned_velocity: NedVelocity,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        """Sends a desired local north, east, down velocity
        """
        self.data.setpoint_latest.ned_velocity = ned_velocity
        self.data.setpoint_latest.yaw = yaw

        self._send_ned_velocity_setpoint(
            ned_velocity=ned_velocity,
            yaw=yaw
        )


    @abstractmethod
    def get_available_fcu_modes(self) -> Tuple[FCUCopterMode]:
        """Some flight controllers may not expose all available Copter modes. This methods returns
        the available modes for a particular flight contoller. 
        """


    def set_fcu_mode(self, mode: FCUCopterMode) -> bool:
        """Sets the flight controller's current mode. Check get_available_fcu_modes first. 

        Returns
        -------
        success
            True if mode switch is successful
        """
        mode_set_successful = self._set_fcu_mode(mode)
        return mode_set_successful

    @property
    @lru_cache(maxsize=None)
    def params(self) -> CopterParameters:
        """Find the flight controller parameters and return them in a CopterParameters object
        """
        return self._load_parameters()

    @abstractmethod
    def _load_parameters(self) -> CopterParameters:
        """Loads the flight controller parameters does any Necessary calculations and creates the CopterParameters object
        """

    @abstractmethod
    def _send_lla_setpoint(
        self,
        lla: LlaPosition,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        """Implements sending a desired global position and yaw to the flight controller
        """


    @abstractmethod
    def _send_ned_setpoint(
        self,
        ned_position: NedPosition,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        """Implements sending a desired local north, east, down position relative to the drone's
        takeoff position
        """


    @abstractmethod
    def _send_ned_velocity_setpoint(
        self,
        ned_velocity: NedVelocity,
        yaw: EnuYaw=EnuYaw(0.0)
    ):
        """Implements sending a desired local north, east, down velocity
        """


    @abstractmethod
    def _set_fcu_mode(self, mode: FCUCopterMode) -> bool:
        """Translates the provided internal FCUCopterMode into a FCU specific command to set the 
        related FCU spefici mode

        Returns
        -------
        success
            True if mode switch is successful
        """


@dataclass
class Data:
    """Stores the latest known data about the drone and its status
    """
    armed: Optional[bool] = False
    """Whether not the flight control unit is armed"""
    arm_position: Optional[TimeStampedLlaPosition] = None
    connected: Optional[bool] = False
    """Whether or not the flight control unit is connected to this program"""
    battery: Optional[Battery] = None
    """The current battery state"""
    attitude: Optional[AttitudeData] = None
    acceleration_linear: Optional[NedAcceleration] = None
    """Acceleration in the local north, east, down frame"""
    velocity: Optional[NedVelocity] = None
    landed_status: FCULandedStatus = FCULandedStatus.UNKNOWN
    """Whether or not the drone is on the ground, in the air, or unknown"""
    heading: Optional[EnuYaw] = None
    """The direction the drone is facing"""
    geofence_status: Optional[bool] = False
    """A flight control unit managed geofence is in place"""
    ground_speed: Optional[float] = None
    heartbeat_status: HeartbeatStatus = HeartbeatStatus.CONTINUE
    location: Optional[PositionDataLla] = None
    relative_altitude: Optional[RelativeAltitude] = None
    mode: FCUCopterMode = FCUCopterMode.UNKNOWN
    setpoint_latest: Optional[SetpointLatest] = None
    """most recent setpoint sent to the flight control unit""" 
    state_name: Optional[str] = None
    """Current active state within the dr-onboard state machine"""
    status: FCUStatus = FCUStatus.UNKNOWN
    """Flight control unit status"""
    target_position: Optional[LlaPosition] = None
    """Target of interest position shared from another drone or the ground control station"""
    uav_id: Optional[str] = None
    """Unique name of the drone which matches its published name for DNS"""
    state_type: Optional[str] = None
    """Current class name for the state within the dr-onboard state machine"""


    def __post_init__(self):
        # we want each new Data instance to have new AttitudeData and PositionDataLla instances
        self.attitude=AttitudeData()
        self.location=PositionDataLla()
        self.setpoint_latest=SetpointLatest()

    def to_dict(self):
        current_drone_attitude = self.attitude.get_attitude()
        current_drone_position = self.location.get_position()
        if current_drone_position:
            current_drone_position = current_drone_position.to_amsl()

        return {
            "uavid": self.uav_id,
            "status": {
                "status": self.status.name,
                "mode": self.mode.name,
                "onboard_pilot": self.state_name,
                # add the class name for the current state
                "state_type": self.state_type,
                "speed": self.ground_speed,
                "location": None if not current_drone_position else {
                    "latitude": current_drone_position.latitude,
                    "longitude": current_drone_position.longitude,
                    "altitude": current_drone_position.altitude,
                },
                "armed": self.armed,
                "battery": None if not self.battery else {
                    "voltage": self.battery.voltage,
                    "current": self.battery.current,
                    "level": self.battery.level,
                },
                "geofence": self.geofence_status,
                "heartbeat_status" : self.heartbeat_status.name,
                "drone_attitude": None if not current_drone_attitude else {
                    "x": current_drone_attitude.x,
                    "y": current_drone_attitude.y,
                    "z": current_drone_attitude.z,
                    "w": current_drone_attitude.w
                },
                "drone_heading": self.heading
            }
        }
