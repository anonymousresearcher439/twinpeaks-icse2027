import sys
from pathlib import Path
import paho.mqtt.publish as publish

drone_names = [
    "Red",
    "Crimson",
    "FireBrick",
    "MidnightBlue",
    "Navy",
    "Fuchsia",
    "Lime",
]


# topic = f"drone/{drone_name}/mission-spec"
mission = "{}"

def send_reset():
    for drone in drone_names:
        topic = f"drone/{drone}/reset"
        # Publish the mission to the MQTT broker
        print(f"Publishing reset to {topic}")
        publish.single(topic, mission, hostname="10.222.0.1", port=1883)
import time
while True:
    print("Sending reset...")
    send_reset()
    time.sleep(5)  # Wait for 5 seconds before sending again
