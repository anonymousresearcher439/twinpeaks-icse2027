import json
import rospy
from typing import Any, Dict, Union

from paho.mqtt.client import MQTTMessage


from dr_onboard_autonomy.models import LlaPosition, DATUM_REFERENCE
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict

class TrajectoryCommander:
    """Component to trigger the state to exit with "task_canceled" outcome when
    we receive a cancel message.
    """

    def __init__(self, state: 'BaseState', **kwargs):
        self.state = state

        # we require `home_altitude_offset` to be passed in as a keyword argument
        if "home_altitude_offset" not in kwargs:
            raise ValueError("TrajectoryCommander requires home_altitude_offset to be passed in as a keyword argument")
        self.home_altitude_offset_amsl = kwargs["home_altitude_offset"] # this is the altitude above sea level of the home location

        # make sure our trajectory is the right type. It must be a WaypointTrajectory
        if not isinstance(state.trajectory, WaypointTrajectory):
            raise ValueError("TrajectoryCommander requires the state's trajectory to be a WaypointTrajectory")
        
        self.trajectory: WaypointTrajectory = state.trajectory

        # finally, add the message sender and handler (now that we've verified all our preconditions)
        state.message_senders.add(state.reusable_message_senders.find("trajectory_command"))
        state.handlers.add_handler("trajectory_command", self.on_trajectory_command, can_transition=True)
    
    def on_trajectory_command(self, msg):
        rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@ Received trajectory command")
        mqtt_msg: MQTTMessage = msg["data"]
        try:
            msg = mqtt_msg.payload.decode("utf-8")
            if "DONE" == msg.strip().upper():
                return "done"
            payload = json.loads(msg)
            rospy.loginfo(f"@@@@@@@@@@@@@@@@@@@@@@ trajectory command: {payload}")

        except UnicodeDecodeError as e:
            rospy.logerr(f"Could not accept trajectory command. Error decoding UTF-8: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            return
        except json.JSONDecodeError as e:
            rospy.logerr(f"Could not accept trajectory command. Error parsing JSON message: {e}\nMessage Payload:'{mqtt_msg.payload}'")
            return
        
        # ensure the payload matches the expected format
        # we need 
        required_entries = [
            ("latitude", float),
            ("longitude", float),
        ]
        for entry, entry_type in required_entries:
            if entry not in payload:
                rospy.logerr(f"Could not accept trajectory command. Missing required entry: {entry}")
                return
            if not isinstance(payload[entry], entry_type):
                rospy.logerr(f"Could not accept trajectory command. Entry {entry} must be of type {entry_type}")
                return
        # we also need a speed entry that can be float or int
        if "speed" not in payload:
            rospy.logerr(f"Could not accept trajectory command. Missing required entry: speed")
            return
        if not isinstance(payload["speed"], (float, int)) or payload["speed"] < 0:
            rospy.logerr(f"Could not accept trajectory command. Entry speed must be a positive number")
            return
        # we also need either "altitude" or "relative_altitude"
        if "altitude" not in payload and "relative_altitude" not in payload:
            rospy.logerr(f"Could not accept trajectory command. Missing required entry: altitude or relative_altitude")
            return
        # if relative_altitude is present, we must figure out the corresponding altitude above sea level
        if 'relative_altitude' in payload:
            if 'altitude' in payload:
                rospy.logwarn("Both altitude and relative_altitude are present. relative_altitude takes priority.")
            altitude = self.home_altitude_offset_amsl + payload['relative_altitude']
            payload['altitude'] = altitude

        # now we can create a LlaPosition object
        waypoint = LlaPosition(
            latitude=payload["latitude"],
            longitude=payload["longitude"],
            altitude=payload["altitude"], 
            datum_ref=DATUM_REFERENCE.AMSL,
        )
        speed_limit = payload["speed"]

        # finally, we can fly to the waypoint
        self.trajectory: WaypointTrajectory = self.state.trajectory
        self.trajectory.fly_to_waypoint(waypoint, speed_limit)


from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory
from dr_onboard_autonomy.states import BaseState