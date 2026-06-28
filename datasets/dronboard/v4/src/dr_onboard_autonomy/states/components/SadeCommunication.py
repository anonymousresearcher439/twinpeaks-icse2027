import json

from dr_onboard_autonomy.models.drone import Battery
from dr_onboard_autonomy.mqtt_client import MQTTClient
import rospy
from typing import Dict, Optional, TypedDict, Union, List

from paho.mqtt.client import MQTTMessage
import rospy

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict


class SADEResponse(TypedDict):
    sade_zone_id: str
    uav_id: str
    pilot_id: str
    decision: str
    timestamp: str
    status_data: Optional[List[str]]


class SadeCommunication:
    """Component to handle communication with the SADE system.
    """

    def __init__(self,
            state: BaseState,
            sade_zone_id: str,
            drone_registration_id: str,
            pilot_id: str,
            owner_id: str,
            model_name: str,
            enable_entry_outcomes: bool,
            enable_exit_outcomes: bool,
        ):
        self.state = state
        self.sade_zone_id = sade_zone_id
        self.drone_registration_id = drone_registration_id
        self.pilot_id = pilot_id
        self.owner_id = owner_id
        self.model_name = model_name
        assert state.mqtt_client is not None, "MQTT client must be initialized in the state."
        assert state.mqtt_cloud is not None, "MQTT cloud client must be initialized"
        self.mqtt_cloud: MQTTClient = state.mqtt_cloud
        self.mqtt_client: MQTTClient = state.mqtt_client
        # self.config = state.config
        assert self.state.reusable_message_senders is not None, "Reusable message senders must be initialized."

        if enable_entry_outcomes:
            self.state.message_senders.add(self.state.reusable_message_senders.find("sade_entry_response"))
            self.state.handlers.add_handler("sade_entry_response", self._on_sade_entry_response)

        if enable_exit_outcomes:
            self.state.message_senders.add(self.state.reusable_message_senders.find("sade_exit_response"))
            self.state.handlers.add_handler("sade_exit_response", self._on_sade_exit_confirmation)

    def send(self, topic: str, payload: Dict):
        self.mqtt_cloud.publish(topic, payload, qos=1)
        self.mqtt_client.publish(topic, payload, qos=1)

    def send_uav_registration(self):
        assert self.state.drone is not None
        payload = {
            "droneID": self.drone_registration_id,
            "pilotID": self.pilot_id,
            "ownerID": self.owner_id,
            "modelName": self.model_name,
            "timestamp": MQTTClient.timestamp(),
        }
        rospy.loginfo(f"SadeCommunication sending UAV registration: {payload}")
        self.send("sade_manager/uav/register_model", payload)
    
    def send_pilot_registration(self):
        # TODO load  pilot registration details from config
        payload = {
            "pilotID": self.pilot_id,
            "droneID": self.drone_registration_id,
            "ownerID": self.owner_id,
            "modelName": self.model_name,
            "timestamp": MQTTClient.timestamp(),
        }
        rospy.loginfo(f"SadeCommunication sending pilot registration: {payload}")
        self.send("sade_manager/uav/register", payload)

    def send_entry_request(self):
        uav_name = self.state.uav_id
        payload = {
            "pilotID": self.pilot_id,
            "droneID": self.drone_registration_id,
            "sade_zone_id": self.sade_zone_id,
            "uavID": self.state.uav_id,
            "response_topic": f"drone/{uav_name}/sade_entry_response",
            "timestamp": MQTTClient.timestamp(),
        }
        rospy.loginfo(f"SadeCommunication sending SADE entry request: {payload}")
        self.send("sade_manager/sade/request_access", payload)
    
    def send_sade_exit(self):
        uav_name = self.state.uav_id
        msg = {
            "pilotID": self.pilot_id,
            "droneID": self.drone_registration_id,
            "sade_zone_id": self.sade_zone_id,
            "response_topic": f"drone/{uav_name}/sade_exit_response",
            "timestamp": MQTTClient.timestamp()
        }
        self.send("sade_manager/sade/exit", msg)

    
    def _on_sade_exit_confirmation(self, msg: Dict):
        """receive a raw response message directly from the Message sender"""
        mqtt_msg = msg['data']
        return self.process_sade_exit_confirmation(mqtt_msg)

    def process_sade_exit_confirmation(self, msg: Union[MQTTMessage, str, Dict]):
        """Handle the confirmation of the SADE exit."""
        if isinstance(msg, MQTTMessage):
            msg = msg.payload.decode("utf-8")
        # if isinstance(msg, str):
        #     try:
        #         msg = json.loads(msg)
        #     except json.JSONDecodeError as e:
        #         rospy.logerr(f"Failed to decode SADE exit confirmation: {e}")
        #         return
        # assert isinstance(msg, dict)
        # TODO look at the message and decide what to do
        rospy.loginfo(f"SadeCommunication Received SADE exit confirmation: {msg}")
        return "success_sade_exit"

    def _on_sade_entry_response(self, msg: Dict):
        """receive a raw response message directly from the Message sender"""
        mqtt_msg = msg['data']
        return self.process_sade_response(mqtt_msg)
    
    def process_sade_response(self, msg: Union[MQTTMessage, str, Dict]):
        rospy.loginfo(f"SadeCommunication processing SADE response: {msg}")
        """Handle the response from the SADE system."""
        if isinstance(msg, MQTTMessage):
            msg = msg.payload.decode("utf-8")
        if isinstance(msg, str):
            try:
                msg = json.loads(msg)
            except json.JSONDecodeError as e:
                rospy.logerr(f"Failed to decode SADE response: {e}")
                return
        assert isinstance(msg, dict)

        # check if we're being asked to update our status message
        if 'fields' in msg:
            fields = msg['fields']
            # make sure this property is a list of strings
            if isinstance(fields, list) and all(isinstance(field, str) for field in fields):
                self.state.status_message.update_status_message(fields)
            else:
                rospy.logwarn(f"SadeCommunication Cannot update status message fields. msg[fields] is invalid: 'msg[fields] = {fields}'. Expected a list of strings")
                return

        rospy.loginfo(f"SadeCommunication Received SADE response: {msg}")
        # make sure the pilot, uav_id, and sade_zone_id are present and that they are relevant to this drone
        #TODO check that the pilot and uav_id match the drone's pilot and uav_id
        # msg['uav_id'] = self.state.uav_id
        # msg['pilot_id'] = self.state.pilot_id
        if msg['sade_zone_id'] != self.sade_zone_id:
            rospy.loginfo(f"SADE response for different zone: {msg['sade_zone_id']} != {self.sade_zone_id}. Ignoring.")
            return
        
        # next we need to check the decision and return the appropriate outcome
        if 'decision' not in msg:
            rospy.logerr("SadeCommunication SADE response does not contain a decision")
            return
        rospy.loginfo(f"SadeCommunication SADE response decision: {msg['decision']}")
        
        if msg['decision'] == 'ALLOW':
            return "allow"
        elif msg['decision'] == 'DENY':
            return "deny"
        elif msg['decision'] == 'PROVING_GROUND':
            return "proving_ground"
        else:
            rospy.logerr(f"SadeCommunication SADE response has unknown decision: {msg['decision']}")
            # ignore the response