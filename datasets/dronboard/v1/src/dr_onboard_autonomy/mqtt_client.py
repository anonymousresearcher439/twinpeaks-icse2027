import json
from threading import Condition, Event, Lock

import paho.mqtt.client as mqtt
import rospy


class MQTTClient:
    def __init__(self, uav_name, broker_address, broker_port=1883, client_id=""):
        self.uav_name = uav_name
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.client_id = client_id

        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.connected_event = Event()

        rospy.on_shutdown(self.disconnect)

    def connect(self):
        self.client.connect(self.broker_address, self.broker_port)
        self.client.loop_start()

    def disconnect(self):
        self.publish("disconnect_drone", {"uavid": self.uav_name})
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        # Return code of 0 specifies a successful connection
        if rc == 0:
            self.connected_event.set()
            print("Connected to MQTT broker successfully!")
        else:
            print("Failed to connect to MQTT broker with return code: {}".format(rc))

    def on_disconnect(self, client, userdata, rc):
        self.connected_event.clear()
        if rc == 0:
            print("Successful disconnection")
        else:
            print("Unexpected disconnection")
        self.client.loop_stop()

    def publish(self, topic, data, qos=2):
        payload = json.dumps(data)
        self.client.publish(topic, payload, qos)

    def arming_status_update(self, message, m_type):
        data = {"uavid": self.uav_name, "data": {"message": message, "type": m_type}}
        self.publish("arming_status_update", data)

    def subscribe(self, topic, callback, qos=0):
        self.client.message_callback_add(topic, callback)
        self.client.subscribe(topic, qos)

    def remove_callback_and_unsubscribe(self, topic):
        self.client.message_callback_remove(topic)
        err, mid = self.client.unsubscribe(topic)
