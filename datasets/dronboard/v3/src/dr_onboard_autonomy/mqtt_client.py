import json
from datetime import datetime, timezone
from threading import Condition, Event, Lock
from typing import Dict, NamedTuple, List

import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTTMessageInfo
import rospy


_SubscriptionCall = NamedTuple(
    "SubscriptionCall", [("topic", str), ("callback", callable), ("qos", int)]
)


class MQTTClient:
    def __init__(self, uav_name: str, broker_address, broker_port=1883):
        assert isinstance(uav_name, str)
        self.uav_name = uav_name
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.subscription_calls: List[_SubscriptionCall] = []
        # client id will be uav_name all uppercase
        self.client_id = self.uav_name.upper()

        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_socket_open = self.on_socket_open
        self.connected_event = Event()
        self.local_ip = None

        rospy.on_shutdown(self.disconnect)

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
                f"Connected to MQTT broker at host: '{self.broker_address}' port: {self.broker_port}!"
            )
            self._resubscribe()
        else:
            rospy.logerr(
                "Failed to connect to MQTT broker with return code: {}".format(rc)
            )

    def on_socket_open(self, _client, _userdata, sock):
        self.local_ip = sock.getsockname()[0]

    def on_disconnect(self, client, userdata, rc):
        self.connected_event.clear()
        if rc == 0:
            rospy.loginfo(
                f"Successful disconnection from host: '{self.broker_address}' port: {self.broker_port}"
            )
        else:
            rospy.logerr(
                f"Unexpected disconnection from host: '{self.broker_address}' port: {self.broker_port}"
            )

    def publish(self, topic: str, data: Dict, qos=2):
        payload = json.dumps(data)
        outcome: MQTTMessageInfo = self.client.publish(topic, payload, qos)
        return outcome

    def arming_status_update(self, message, m_type):
        data = {"uavid": self.uav_name, "data": {"message": message, "type": m_type}}
        self.publish("arming_status_update", data)

    def subscribe(self, topic, callback, qos=2):
        self.subscription_calls.append(_SubscriptionCall(topic, callback, qos))
        self._subscribe_internal(topic, callback, qos)

    def _subscribe_internal(self, topic, callback, qos=2):
        self.client.message_callback_add(topic, callback)
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
