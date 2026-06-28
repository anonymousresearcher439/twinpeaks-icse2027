import json
from typing import List, Optional
from typing import Tuple

from dr_onboard_autonomy.airlease.protocol import AirLeaserModel
import rospy
from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.logutils import DebounceLogger

class StatusMessage:
    """Component to build and send status messages over MQTT
    """
    def __init__(self, state: 'BaseState', frequency: float = 1.0):
        """
        Args:
            state (BaseState instance): The state that this component is a part of
            frequency (float): How often to send the status message
        """
        assert state is not None, "State must be provided to StatusMessage"
        assert state.mqtt_client is not None, "MQTT client must be provided to StatusMessage"
        assert state.local_mqtt_client is not None, "Local MQTT client must be provided to StatusMessage"
        
        self.state: 'BaseState' = state
        assert self.state.reusable_message_senders is not None, "Reusable message senders must be provided to StatusMessage"
        self._protocol_fsm: Optional[AirLeaserModel] = state.air_lease_protocol_fsm
        self.debounce_logger = DebounceLogger()
        TIMER_NAME = "StatusMessage__send_status_timer"
        self.state.message_senders.add(RepeatTimer(TIMER_NAME, frequency))
        self.state.handlers.add_handler(TIMER_NAME, self._send_status_message)
        required_msg_senders = [
            "battery",
            "imu",
            "compass_hdg",
            "velocity",
            "position",
            "relative_altitude",
            "state",
            "gps_status",
            "target_attitude",
            "vibration",
            "rc_out",
            # "ekf_status",
        ]
        for sender in required_msg_senders:
            self.state.message_senders.add(self.state.reusable_message_senders.find(sender))
        
        self._data = state.data
        self._OPTIONAL_FIELDS_KEY = type(self).__name__ + "_optional_fields"
        self.state.message_senders.add(self.state.reusable_message_senders.find("update_status_message"))
        self.state.handlers.add_handler("update_status_message", self.on_update_status_message_command)

        # setup outputs
        self.output_clients = [
            (self.state.mqtt_client, "update_drone"),
            (self.state.local_mqtt_client, "update_drone"),
        ]
        # the cloud MQTT client is optional
        if state.mqtt_cloud is not None:
            self.output_clients.append((self.state.mqtt_cloud, "status_message"))
    
    @property
    def optional_fields(self) -> List[str]:
        return self._data.get(self._OPTIONAL_FIELDS_KEY, [])
    
    @optional_fields.setter
    def optional_fields(self, fields: List[str]) -> None:
        self._data[self._OPTIONAL_FIELDS_KEY] = fields

    def on_update_status_message_command(self, message: dict) -> None:
        """
        Receive a command to update the status message with new fields.

        The dict should have two properties:
        - "type": "update_status_message"
        - "data": an MQTTMessage

        The MQTTMessage payload should be a JSON string with the following structure:
        ```json
        {
            "fields": ["field1", "field2"]
        }
        ```
        """
        mqtt_message: MQTTMessage = message.get("data", {})
        response = mqtt_message.payload.decode("utf-8")
        # log that we received the command
        rospy.loginfo(f"Received update status message command: {response}")
        # try and parse the JSON
        try:
            json_data = json.loads(response)
            fields = json_data.get("fields", [])
            if isinstance(fields, list):
                self.update_status_message(fields)
            else:
                rospy.logwarn(f"Invalid fields format: {fields}. Expected a list.")
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to decode JSON from update status message command: {response}. Error: {e}")

    def update_status_message(self, fields: List[str]) -> None:
        """
        Update the status message with the given fields.

        Args:
            fields (List[str]): List of fields to include in the status message.
        """
        self.optional_fields = fields

    def _send_status_message(self, message: dict):
        assert self.state.drone is not None
        assert self.state.drone.data is not None
        assert self.state.drone.gimbal is not None
        current_gimbal_attitude = self.state.drone.gimbal.get_attitude()

        output = self.state.drone.data.to_dict()
        assert self.state.mqtt_client is not None
        output.update({
            "timestamp": self.state.mqtt_client.timestamp(),
        })
        output_status: dict = output["status"]
        output_status.update({
            "gimbal_attitude": None if not current_gimbal_attitude else {
                "x": current_gimbal_attitude.x,
                "y": current_gimbal_attitude.y,
                "z": current_gimbal_attitude.z,
                "w": current_gimbal_attitude.w
            },
            "gimbal_heading" : self.state.drone.gimbal.heading,
        })
        # indicate if we're waiting for an air lease
        if self._protocol_fsm is not None:
            current_state = self._protocol_fsm.state.name # type: ignore
            output_status["air_lease_state"] = current_state
        
        # now we add the optional fields
        extra_data = self.state.drone.data.optional_data()
        for field in self.optional_fields:
            if field in extra_data:
                value = extra_data[field]
                # Convert known objects to JSON-serializable dicts when possible
                if value is None:
                    output_status[field] = None
                else:
                    output_status[field] = value.to_dict()  # type: ignore[attr-defined]
        self.debounce_logger.info(f"StatusMessage: sending status message: {output}", 1.5, 999)
        
        for client, topic in self.output_clients:
            if client is not None:
                client.publish(topic, output, qos=0)
