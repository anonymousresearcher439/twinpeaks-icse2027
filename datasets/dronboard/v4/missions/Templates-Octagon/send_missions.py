import argparse
import json
from pathlib import Path
from string import Template
import time
# use paho mqtt client to send the missions
import paho.mqtt.client as mqtt



# Parse required positional 'speed' argument
parser = argparse.ArgumentParser(
    description="Send missions in this folder with the provided speed injected into templates."
)
parser.add_argument(
    "speed",
    type=float,
    help="Mission speed as a float (e.g., 5.0)",
)
parser.add_argument(
    "--mqtt-host",
    default="10.222.0.1",
    help="MQTT broker host (default: 10.222.0.1)",
)
parser.add_argument(
    "--mqtt-port",
    type=int,
    default=1883,
    help="MQTT broker port (default: 1883)",
)
args = parser.parse_args()


mqtt_client = mqtt.Client()
mqtt_client.connect(args.mqtt_host, args.mqtt_port)

mqtt_client.loop_start()
while not mqtt_client.is_connected():
    time.sleep(0.001)


print("TESTING 123")
# I need to get the directory that this file is within
current_file_path = Path(__file__).resolve()
print(current_file_path)
current_directory = current_file_path.parent
print(current_directory)
print(f"MQTT broker: {args.mqtt_host}:{args.mqtt_port}")

# next lets find all the *-mission.json
mission_files = list(current_directory.glob("*-mission.json.tmpl"))

# we use the file name to figure out the drone name
# for example Aqua-mission.json == Aqua
drone_names = [file.stem.split("-")[0] for file in mission_files]

# zip the two together
drone_mission_pairs = zip(drone_names, mission_files)
for drone_name, mission_file in drone_mission_pairs:
    # Now you can use drone_name and mission_file
    print(f"Sending mission for drone: {drone_name} using file: {mission_file}")
    # before we send the mission we need to use the python
    # template lib to update the "speed"
    template_text = Path(mission_file).read_text()
    mission_template = Template(template_text)
    mission = mission_template.substitute(speed=args.speed)
    topic = f"drone/{drone_name}/mission-spec"
    print(f"Sending mission to drone: '{drone_name}' with updated speed: {args.speed}, TOPIC='{topic}'")

    mqtt_client.publish(topic, mission, qos=1)

mqtt_client.loop_stop()
