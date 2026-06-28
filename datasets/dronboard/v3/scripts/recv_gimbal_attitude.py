#!/usr/bin/env python
# This script connects to the GCS MQTT server
# It listens for messages on the "update_drone" topic
# It prints a `.` every time it receives a message without gimbal data.
# (This is to show that it's receiving messages.)
# Otherwise, it prints the gimbal attitude and heading if they are present in the message.
USAGE = """
Usage:
    python recv_gimbal_attitude.py [MQTT_HOST] [MQTT_PORT]
"""
import paho.mqtt.client as mqtt
import json
import sys

if len(sys.argv) > 1 and (sys.argv[1] == "-h" or sys.argv[1] == "--help" or sys.argv[1] == "help"):
    print(USAGE)
    exit(0)

print("starting script")
mqtt_client = mqtt.Client(client_id="RECV-GIMBAL-DATA-SCRIPT")
# matt-lab-jetson.local
mqtt_host = sys.argv[1] if len(sys.argv) > 1 else "host.docker.internal"
mqtt_port = sys.argv[2] if len(sys.argv) > 2 else 1883
mqtt_port = int(mqtt_port)
if len(sys.argv) < 2:
    print("Using default MQTT host and port")

print(f"Connecting to {mqtt_host} on port {mqtt_port}")
mqtt_client.connect(mqtt_host, mqtt_port)

def on_update_drone(client: mqtt.Client, userdata, message: mqtt.MQTTMessage):
    #print(f"Received message on topic {message.topic}")
    #print(f"Message: {message.payload}")
    print(".", end="", flush=True) # this is to show that we're receiving messages
    json_data = json.loads(message.payload)
    gimbal_attitude = json_data['status']['gimbal_attitude']
    gimbal_heading = json_data['status']['gimbal_heading']
    if gimbal_attitude:
        print(f"\nGimbal Attitude: {gimbal_attitude}")
    if gimbal_heading:
        print(f"\nGimbal Heading: {gimbal_heading}")


mqtt_client.message_callback_add("update_drone", on_update_drone)
mqtt_client.subscribe("update_drone", 0)

mqtt_client.loop_forever()