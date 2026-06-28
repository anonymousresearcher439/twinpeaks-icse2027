import rospy

from math import nan
from typing import Dict, Optional, Union

_state_to_display = {
    "UNKNOWN": "UNKNOWN",
    "OnGroundAndInactive": "OnGroundAndInactive",
    "MissionPreparation": "UAV Configured for Mission",
    "Preflight": "Preflight Checks",
    "Arm": "Arming",
    "Takeoff": "Taking Off",
    "FlyToDestination": "Flying to Destination",
    "FlyWaypoints": "Flying to Destination",
    "Hover": "Hover",
    "Searching": "Searching",
    "UserVictimCheck": "User confirmation",
    "VictimDetected": "Victim Detected",
    "Tracking": "Tracking",
    "ReturnHome": "ReturnHome",
    "Land": "Landing",
    "Disarm": "Disarming",
    "PositionForDrop": "Positioning for Drop",
    "Dropping": "Dropping",
}


class Data:
    def __init__(self):
        self.uavid = None
        self.color = "green"
        self.ip = "14550"
        self.status = 0
        self.airspeed = None
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

    def to_dict(self):
        if self.state_name in _state_to_display:
            state_display_text = _state_to_display[self.state_name]
        else:
            state_display_text = self.state_name
            rospy.logwarn(f"Missing display text for state {self.state_name}.")
        drone_response_data = {
            "uavid": self.uavid,
            "color": self.color,
            "ip": self.ip,
            "status": {
                "status": self.status_mapping[self.status],
                "mode": self.mode,
                "onboard_pilot": self.state_name,
                "onboard_pilot2": state_display_text,
                "airspeed": self.airspeed,
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
                "drone_attitude": self.drone_attitude,
                "drone_heading": self.drone_heading,
            },
        }

        return drone_response_data
