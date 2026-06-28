"""Houses all data models and abstract objects
"""
from .config import DRConfig
from .connection import ExternalConnection
from .drone import (
    Battery,
    CopterDrone,
    FCUState,
    FCUStatus,
    FCUCopterMode,
    FCU_MODES_HUMAN_CONTROL,
    HeartbeatStatus
)
from .frames import AttitudeData, PositionData, PositionDataLla
from .gimbal import Gimbal
from .kinematics import (
    ALTITUDE_REFERENCE,
    EnuYaw,
    LlaPosition,
    NedAcceleration,
    NedPosition,
    NedVelocity,
    Quaternion,
    IMU
)