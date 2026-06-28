from typing import Literal, Union

# https://docs.pydantic.dev/latest/
from pydantic import BaseModel, Field


class PX4Gimbal(BaseModel):
    """Gimbal specific configuration"""

    pitch: bool = Field(default=...)
    roll: bool = Field(default=...)
    yaw: bool = Field(default=...)
    type: Literal["px4"] = Field(default="px4")


class GremsyGimbal(BaseModel):
    """Gimbal specific configuration"""

    pitch: bool = Field(default=...)
    roll: bool = Field(default=...)
    yaw: bool = Field(default=...)
    type: Literal["gremsy"] = Field(default="gremsy")


class MQTT(BaseModel):
    host: str = Field(default="mqtt")
    port: int = Field(default=1883, ge=1025, le=65356)


class DRConfig(BaseModel):
    """Root of configuration file format"""

    name: str = Field(default=...)
    system_id: int = Field(default=...)
    performance_analysis: bool = Field(default=False)
    comment: str = Field(default="")
    mqtt: MQTT = Field()
    gimbal: Union[GremsyGimbal, PX4Gimbal] = Field(default=..., discriminator="type")
