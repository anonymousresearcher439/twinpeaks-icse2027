from droneresponse_mathtools import Lla
import jinja2


home = Lla(43.22996220438573, -75.41114918021859, 146.2339)
stare_position = Lla(43.232618, -75.414642, 147)

final_waypoint = home.move_ned(0, 0, -50)

data = {
    "home": home,
    "final_waypoint": final_waypoint,
    "stare_position": stare_position,
}

template = """{
    "HOME": {
        "latitude": {{ home.latitude }},
        "longitude": {{ home.longitude }},
        "altitude": {{ home.altitude }}
    },
    "STARE_POSITION": {
        "latitude": {{ stare_position.latitude }},
        "longitude": {{ stare_position.longitude }},
        "altitude": {{ stare_position.altitude }}
    },
    "states": [
        {
            "name": "MISSION_PREPARATION",
            "transitions": [
                {
                    "target": "OnGround",
                    "condition": "MissionConfigured"
                }
            ]
        },

        {
            "name": "Arm",
            "transitions": [
                {
                    "target": "Takeoff",
                    "condition": "succeeded_armed"
                }
            ]
        },
        {
            "name": "Takeoff",
            "transitions": [
                {
                    "target": "BetterHover",
                    "condition": "succeeded_takeoff"
                },
                {
                    "target": "Land",
                    "condition": "failed_takeoff"
                }
            ]
        },
        {
            "name": "BetterHover",
            "args": {
                "stare_position": {
                    "latitude": {{ stare_position.latitude }},
                    "longitude": {{ stare_position.longitude }},
                    "altitude": {{ stare_position.altitude }}
                },
                "pitch": 45,
                "distance": 170,
                "starting_angle": 90,
                "cruising_altitude": 54.864,
                "speed": 5.0,
                "hover_time": 30.0
            },
            "transitions": [
                {
                    "condition": "succeeded_hover",
                    "target": "BriarTravel"
                }
            ]
        },
        {
            "name": "BriarTravel",
            "args": {
                "waypoint": {
                    "latitude": {{ final_waypoint.latitude }},
                    "longitude": {{ final_waypoint.longitude }},
                    "altitude": {{ final_waypoint.altitude }}
                },
                "stare_position": {
                    "latitude": {{ stare_position.latitude }}, 
                    "longitude": {{ stare_position.longitude }},
                    "altitude": {{ stare_position.altitude }}
                },
                "cruising_altitude": 54.864,
                "speed": 5.0
            },
            "transitions": [
                {
                    "condition": "succeeded_waypoints",
                    "target": "Land"
                }
            ]
        },
        {
            "name": "Land",
            "transitions": [
                {
                    "target": "Disarm",
                    "condition": "succeeded_land"
                }
            ]
        },
        {
            "name": "Disarm",
            "transitions": [
                {
                    "target": "mission_completed",
                    "condition": "succeeded_disarm"
                }
            ]
        }
    ]
}
"""

from jinja2 import Environment, BaseLoader

rtemplate = Environment(loader=BaseLoader()).from_string(template)
print(rtemplate.render(**data))