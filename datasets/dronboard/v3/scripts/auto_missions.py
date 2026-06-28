#!/usr/bin/env python
print("Starting auto missions script")
from pathlib import Path
import paho.mqtt.client as mqtt
from typing import Dict, NamedTuple, List
from threading import Condition, Event, Lock
import json
import signal


mission_directories = [
    Path("./missions/Octagon"),
    Path("./missions/peppermint"),
]
order = [
    -1,
    False,
]
mission_files = []
for i, directory in enumerate(mission_directories):
    dir_files = []
    for path in directory.rglob("*.json"):
        dir_files.append(path)
    if order[i] == -1:
        dir_files.sort(reverse=True)
    mission_files.extend(dir_files)


_SubscriptionCall = NamedTuple(
    "SubscriptionCall", [("topic", str), ("callback", callable), ("qos", int)]
)


class MQTTClient:
    def __init__(self, broker_address, broker_port=1883):

        self.broker_address = broker_address
        self.broker_port = broker_port
        self.subscription_calls: List[_SubscriptionCall] = []
        self.client_id = "AUTO-MISSIONS-SCRIPT"

        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_socket_open = self.on_socket_open
        self.connected_event = Event()
        self.local_ip = None

    def connect(self):
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect_async(self.broker_address, self.broker_port)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        # Return code of 0 specifies a successful connection
        if rc == 0:
            self.connected_event.set()
            print(f"Connected to MQTT broker at host: '{self.broker_address}' port: {self.broker_port}")
            self._resubscribe()
        else:
            print(f"Failed to connect to MQTT broker with return code: {rc}")

    def on_socket_open(self, _client, _userdata, sock):
        self.local_ip = sock.getsockname()[0]

    def on_disconnect(self, client, userdata, rc):
        self.connected_event.clear()
        if rc == 0:
            print(f"Successful disconnection from host: '{self.broker_address}' port: {self.broker_port}")
        else:
            print(f"Unexpected disconnection from host: '{self.broker_address}' port: {self.broker_port}")

    def publish(self, topic: str, data: Dict, qos=2):
        payload = json.dumps(data)
        self.client.publish(topic, payload, qos)

    def arming_status_update(self, message, m_type):
        data = {"uavid": self.uav_name, "data": {"message": message, "type": m_type}}
        self.publish("arming_status_update", data)

    def subscribe(self, topic, callback, qos=0):
        self.subscription_calls.append(_SubscriptionCall(topic, callback, qos))
        self._subscribe_internal(topic, callback, qos)

    def _subscribe_internal(self, topic, callback, qos=0):
        self.client.message_callback_add(topic, callback)
        self.client.subscribe(topic, qos)

    def _resubscribe(self):
        for call in self.subscription_calls:
            self._subscribe_internal(call.topic, call.callback, call.qos)


mqtt_broker = "host.docker.internal"
mqtt_port = 1883


mission_index = 0
def next_mission():
    global mission_index
    global mission_files

    if mission_index >= len(mission_files):
        return None
    mission_file = mission_files[mission_index]
    mission_index += 1
    return mission_file

def load_mission(mission_file):
    with open(mission_file, "r") as f:
        return f.read()

client = MQTTClient(mqtt_broker, mqtt_port)

def send_mission(client, userdata, data):
    print("Received new drone message")
    msg = json.loads(data.payload)
    drone_id = msg.get("uavid")
    if not drone_id:
        print("No drone id in new drone message...")
        return
    
    # try to send the next mission
    mission_file = next_mission()
    if mission_file is None:
        # we need to exit
        MQTTClient.disconnect()
        exit(0)
    mission = load_mission(mission_file)
    if mission is None:
        print(f"Failed to load mission from {mission_file}")
        return
    
    topic= f"drone/{drone_id}/mission-spec"
    print(f"Sending {mission_file} to {drone_id}")
    client.publish(topic, mission)

client.subscribe("new_drone", send_mission)
client.connect()
client.connected_event.wait()
print(f"Connected to broker: {mqtt_broker} on port: {mqtt_port}")
print("Waiting for new drones...")
# main thread will wait for keyboard interrupt or other signal to exit
signal.signal(signal.SIGINT, client.disconnect)
signal.signal(signal.SIGTERM, client.disconnect)


# the main thread needs to wait for the client to disconnect
# try and read a letter from the user to exit
input("Press enter to exit")
client.disconnect()
