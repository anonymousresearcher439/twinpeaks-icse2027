import rospy
from typing import Dict, Optional, Union

from dr_onboard_autonomy.briar_helpers import BriarLla


class Data:
    def __init__(self):
        self.uavid = None
        self.ip = "0.0.0.0"
        self.status = 0
        self.speed = None
        self.location: Dict[str, Optional[float]] = {
            "latitude": None, "longitude": None, "altitude": None
        }
        self.armed_state = None
        self.battery: Dict[str, Optional[float]] = {"voltage": None, "current": None, "level": None}
        self.geofence_status = False
        self.mode = "UNKNOWN"
        self.state_name = "UNKNOWN"
        self.heartbeat_status: Union[str, None] = None
        self.gimbal_attitude: Dict[str, Optional[float]] = {
            "x": None, "y": None, "z": None,"w": None
        }
        self.gimbal_heading: Optional[float] = None
        self.gimbal_pitch: Optional[float] = None
        self.drone_attitude: Dict[str, Optional[float]] = {
            "x": None, "y": None, "z": None,"w": None
        }
        self.drone_heading: Optional[float] = None

        self.status_mapping = {
            0: "Uninitialized",
            1: "Booting up",
            2: "Calibrating",
            3: "Standby",
            4: "Active",
            5: "Critical",
            6: "Emergency",
            7: "Powering off",
            8: "Terminating",
        }

        self.target_position: Optional[BriarLla] = None

    def to_dict(self):

        drone_response_data = {
            "uavid": self.uavid,
            "ip": self.ip,
            "status": {
                "status": self.status_mapping[self.status],
                "mode": self.mode,
                "onboard_pilot": self.state_name,
                "speed": self.speed,
                "location": {
                    "latitude": self.location["latitude"],
                    "longitude": self.location["longitude"],
                    "altitude": self.location["altitude"],
                },
                "armed": self.armed_state,
                "battery": {
                    "voltage": self.battery["voltage"],
                    "current": self.battery["current"],
                    "level": self.battery["level"],
                },
                "geofence": self.geofence_status,
                "heartbeat_status" : self.heartbeat_status,
                "gimbal_attitude" : self.gimbal_attitude,
                "gimbal_heading" : self.gimbal_heading,
                "gimbal_pitch" : self.gimbal_pitch,
                "drone_attitude": self.drone_attitude,
                "drone_heading": self.drone_heading,
            },
        }

        return drone_response_data
