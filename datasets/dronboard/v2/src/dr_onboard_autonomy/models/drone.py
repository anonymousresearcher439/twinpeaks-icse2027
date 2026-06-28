from abc import ABC, abstractmethod
from dataclasses import dataclass
import enum
from typing import NamedTuple, Optional, Tuple, Union

from .frames import AttitudeData, PositionDataLla
from .gimbal import Gimbal
from .kinematics import EnuYaw, LlaPosition, NedPosition, NedVelocity, Quaternion


class Battery(NamedTuple):
    voltage: float
    current: float
    level: float


class FCUCopterMode(enum.Enum):
    ALT_HOLD = enum.auto()
    """Holds the altitude only. Drone will drift horizontally wiht outside forces"""
    LAND = enum.auto()
    """Flight controller alone immediately manages landing the vehicle"""
    LOITER = enum.auto()
    """Flight controller holds current altitude and position with no response to stick movements"""
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
    FCUCopterMode.POS_HOLD.value,
    FCUCopterMode.RTL.value,
    FCUCopterMode.STABILIZE.value
))


@dataclass
class FCUState:
    """Holds data related to the flight controller state
    """
    mode: FCUCopterMode


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


class CopterDrone(ABC):
    def __init__(self, gimbal: Optional[Gimbal], uav_name: str):
        self.uav_name = uav_name
        self.gimbal = gimbal

        self.data = Data()
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


    def get_fcu_mode(self) -> Optional[FCUCopterMode]:
        """Returns the flight controller's current mode
        """
        return self._fcu_mode


    def set_fcu_mode(self, mode: FCUCopterMode) -> bool:
        """Sets the flight controller's current mode. Check get_available_fcu_modes first. 

        Returns
        -------
        success
            True if mode switch is successful
        """
        mode_set_successful = self._set_fcu_mode(mode)

        if mode_set_successful:
            self._fcu_mode = mode

        return mode_set_successful


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
    battery: Optional[Battery] = None
    attitude: Optional[AttitudeData] = None
    heading: Optional[EnuYaw] = None
    geofence_status: Optional[bool] = False
    """A flight control unit managed geofence is in place"""
    ground_speed: Optional[float] = None
    heartbeat_status: HeartbeatStatus = HeartbeatStatus.CONTINUE
    location: Optional[PositionDataLla] = None
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


    def __post_init__(self):
        # we want each new Data instance to have new AttitudeData and PositionDataLla instances
        self.attitude=AttitudeData()
        self.location=PositionDataLla()
        self.setpoint_latest=SetpointLatest()
