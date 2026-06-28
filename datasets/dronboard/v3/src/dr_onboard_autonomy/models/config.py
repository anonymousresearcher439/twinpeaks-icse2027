import math
from enum import Enum
from typing import Literal, Union, Optional

# https://docs.pydantic.dev/latest/
from pydantic import BaseModel, Field,  model_validator, computed_field


class GimbalLimits(BaseModel):
    min: float = Field(ge=-180.0, le=180.0, default=...)
    max: float = Field(ge=-180.0, le=180.0, default=...)

    @model_validator(mode='after')
    def check_min_less_than_max(self):
        if self.max <= self.min:
            raise ValueError(f"min value {self.min} must be less than max value {self.max}")
        return self

class GimbalConfig(BaseModel):
    """Gimbal specific configuration"""

    pitch: bool = Field(default=...)
    roll: bool = Field(default=...)
    yaw: bool = Field(default=...)

    pitch_limits: Optional[GimbalLimits] = None
    roll_limits: Optional[GimbalLimits] = None
    yaw_limits: Optional[GimbalLimits] = None
    

GimbalAxes = Literal["three_axis", "roll_first", "pitch_first"]


class MavlinkGimbalManager(GimbalConfig):
    """Gimbal specific configuration"""
    type: Literal["mavlink_gimbal_manager"] = Field(default="mavlink_gimbal_manager")


class GremsyGimbal(GimbalConfig):
    """Gimbal specific configuration"""
    type: Literal["gremsy"] = Field(default="gremsy")
    axes: GimbalAxes = Field(default=None)

    @model_validator(mode='after')
    def set_axes(self):
        if self.axes is None:
            if self.pitch and self.roll and self.yaw:
                self.axes = "three_axis"
            elif self.roll and self.pitch:
                self.axes = "roll_first"
        return self


class FixedCameraGimbal(BaseModel):
    """A fixed camera attached to the drone.
    
    The angles are in degrees.
    TODO decide if the frame of reference is Forward-Left-Up or Forward-Right-Down
    """
    type: Literal["fixed_camera"] = Field(default="fixed_camera")

    # TODO document the axes of rotaiton
    # TODO document the order that rotations are applied
    # NOTE: The intention is for (1) the yaw rotation to appliy first, (2) the pitch rotation to apply next (in the mobile frame) and (3) the roll rotation to apply last (also in the mobile frame).
    yaw: float = Field(default=0.0)
    pitch: float = Field(default=0.0)
    roll: float = Field(default=0.0)
    # TODO allow us to specify a quaternion instead of euler angles


class CameraConfig(BaseModel):
    """Camera specific configuration"""
    horizontal_fov: float = Field(default=...)
    vertical_fov: float = Field(default=...)


class GimbalAxesConfig(BaseModel):
    type: Literal["GimbalAxisConfig"] = Field(default="GimbalAxisConfig")


class MQTT(BaseModel):
    host: str = Field(default="mqtt")
    port: int = Field(default=1883, ge=1025, le=65356)


class ArdupilotFCU(BaseModel):
    type: Literal["ardupilot"] = Field(default="ardupilot")


class Px4FCU(BaseModel):
    type: Literal["px4"] = Field(default="px4")


class DRConfig(BaseModel):
    """Root of configuration file format"""

    name: str = Field(default=...)
    system_id: int = Field(default=...)
    performance_analysis: bool = Field(default=False)
    comment: str = Field(default="")
    mqtt: MQTT = Field()
    mqtt_local: MQTT = Field()
    gimbal: Union[GremsyGimbal, MavlinkGimbalManager, FixedCameraGimbal] = Field(default=..., discriminator="type")
    fcu: Union[ArdupilotFCU, Px4FCU] = Field(default=..., discriminator="type")
    camera: Optional[CameraConfig] = Field(default=None)
