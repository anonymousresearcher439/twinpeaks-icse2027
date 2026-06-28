import json
from datetime import datetime, timezone
from threading import Condition, Event, Lock
from typing import Dict, NamedTuple, List, Optional
import ssl 

import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTTMessageInfo
import rospy

from dr_onboard_autonomy.models.config import MQTTMutualTLS
from dr_onboard_autonomy.models.config import MQTT as MQTTConfig


_SubscriptionCall = NamedTuple(
    "SubscriptionCall", [("topic", str), ("callback", callable), ("qos", int)]
)


class MQTTClient:
    def __init__(self, uav_name: str, broker_address, broker_port=1883, tls: Optional[MQTTMutualTLS] = None, client_id: Optional[str] = None, max_qos: int = 2):
        assert isinstance(uav_name, str)
        self.uav_name = uav_name
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.max_qos = max_qos

        self.subscription_calls: List[_SubscriptionCall] = []
        
        self.client_id = client_id
        if not self.client_id:
            # if the user doesn't require a client id,
            # then the client id will be uav_name all uppercase
            self.client_id = self.uav_name

        self.client = mqtt.Client(client_id=self.client_id)
        if tls:
            rospy.loginfo(f"MQTT client_id={self.client_id} using mutual TLS with broker at {broker_address} port {broker_port}")
            self.client.tls_set(
                ca_certs=tls.ca_cert,
                certfile=tls.public_cert,
                keyfile=tls.private_key,
                tls_version=ssl.PROTOCOL_TLS,
                ciphers=None
            )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_socket_open = self.on_socket_open
        self.connected_event = Event()
        self.local_ip = None

        rospy.on_shutdown(self.disconnect)
    
    @staticmethod
    def from_config(uav_name: str, config: MQTTConfig) -> "MQTTClient":
        """Create an MQTTClient instance from a configuration object.
        
        Args:
            uav_name (str): The name of the UAV. THIS IS REQUIRED. This cannot be an empty string.
            config (MQTTConfig): The configuration object containing MQTT settings.
        """
        assert isinstance(config, MQTTConfig)
        assert isinstance(uav_name, str)
        assert len(uav_name) > 0, "UAV name must be a non-empty string"
        return MQTTClient(
            uav_name=uav_name,
            broker_address=config.host,
            broker_port=config.port,
            tls=config.tls,
            client_id=config.client_id,
            max_qos=config.max_qos
        )

    def connect(self):
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect_async(self.broker_address, self.broker_port, keepalive=5)
        self.client.loop_start()

    def disconnect(self):
        self.publish("disconnect_drone", {"uavid": self.uav_name})
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        # Return code of 0 specifies a successful connection
        if rc == 0:
            self.connected_event.set()
            rospy.loginfo(
                f"MQTT Connected to MQTT broker at host: '{self.broker_address}' port: {self.broker_port}!"
            )
            self._resubscribe()
        else:
            rospy.logerr(
                f"MQTT Failed to connect to MQTT broker with return code: {rc}"
            )

    def on_socket_open(self, _client, _userdata, sock):
        self.local_ip = sock.getsockname()[0]

    def on_disconnect(self, client, userdata, rc):
        self.connected_event.clear()
        if rc == 0:
            rospy.loginfo(
                f"MQTT Successful disconnection from host: '{self.broker_address}' port: {self.broker_port}"
            )
        else:
            rospy.logerr(
                f"MQTT Unexpected disconnection from host: '{self.broker_address}' port: {self.broker_port}"
            )

    def publish(self, topic: str, data: Dict, qos=None):
        if qos is None:
            qos = self.max_qos
        if qos > self.max_qos:
            rospy.logwarn(
                f"MQTT: Provided QoS {qos} is greater than the max QoS {self.max_qos} supported by the broker at {self.broker_address}:{self.broker_port} (client_id={self.client_id}). Using max QoS {self.max_qos} instead."
            )
            qos = self.max_qos
        payload = json.dumps(data)
        rospy.loginfo(f"MQTTClient publishing to topic '{topic}' with QoS {qos}: {payload}")
        outcome: MQTTMessageInfo = self.client.publish(topic, payload, qos)
        return outcome

    def send_error(self, subject: str, message: str):
        """Send an error message to the MQTT server."""
        rospy.logerr(f"Error in MQTTClient: {message}")
        error_msg = {
            "uavid": self.uav_name,
            "timestamp": self.timestamp(),
            "alert": subject,
            "message": message,
        }
        self.publish(f"drone/{self.uav_name}/error", error_msg, qos=1)

    def arming_status_update(self, message, m_type):
        data = {"uavid": self.uav_name, "data": {"message": message, "type": m_type}}
        self.publish("arming_status_update", data)

    def subscribe(self, topic, callback, qos=2):
        self.subscription_calls.append(_SubscriptionCall(topic, callback, qos))
        self._subscribe_internal(topic, callback, qos)

    def _subscribe_internal(self, topic, callback, qos=2):
        rospy.loginfo(f"MQTTClient subscribing to topic '{topic}' with QoS {qos}")
        self.client.message_callback_add(topic, callback)
        if qos > self.max_qos:
            rospy.logwarn(
                f"MQTT: Provided QoS {qos} is greater than the max QoS {self.max_qos} supported by the broker at {self.broker_address}:{self.broker_port} (client_id={self.client_id}). Using max QoS {self.max_qos} instead."
            )
            qos = self.max_qos
        self.client.subscribe(topic, qos)

    def _resubscribe(self):
        for call in self.subscription_calls:
            self._subscribe_internal(call.topic, call.callback, call.qos)

    # TODO: implement this if we need it
    # all subscriptions we make are stored in self.subscription_calls
    # so we'll need to remove the subscription from that list
    # and unsubscribe from the broker
    # def remove_callback_and_unsubscribe(self, topic):
    #     self.client.message_callback_remove(topic)
    #     err, mid = self.client.unsubscribe(topic)

    def is_connected(self):
        return self.connected_event.is_set()

    # I need to create a function that returns a timestamp string in ISO format
    # something like '2024-02-08T01:51:49.111+00:00'
    # this string needs be included in my message to the GCS
    # I will use the `datetime` module to create this function
    @staticmethod
    def timestamp() -> str:
        """A timestamp in ISO format with milliseconds and UTC timezone.

        It's a best practice to include a timestamp in messages. This function
        returns a timestamp string in ISO format with milliseconds and UTC timezone.

        Here's an example timestamp:

            '2024-02-08T02:15:55.512+00:00'

        Timestamps in this format are easy to work with. For example, in
        JavaScript you can create a Date object from this string like so:

            const d = new Date('2024-02-08T02:15:55.512+00:00')

        The object `d` will represent the date and time specified by the timestamp string.

        In Python you can also create a datetime object from this string like so:

            from datetime import datetime
            d = datetime.fromisoformat('2024-02-08T02:15:55.512+00:00')

        The object `d` will represent the date and time specified by the timestamp string.

        Returns:
            str: A timestamp in ISO format with milliseconds and UTC timezone.
        """
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds")
