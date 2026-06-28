import paho.mqtt.client as mqtt
import csv
import argparse
import os
import signal
import sys
import time

# Graceful exit handler
def signal_handler(sig, frame):
    print("\nExiting... Closing CSV file.")
    csv_file.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Parse command line arguments
parser = argparse.ArgumentParser(description="MQTT Subscriber to CSV")
parser.add_argument("host", help="MQTT broker host")
parser.add_argument("topics", nargs='+', help="MQTT topics to subscribe to (space-separated list)")
parser.add_argument("-o", "--output", default="output.csv", help="Output CSV file name (default: output.csv)")

args = parser.parse_args()

t0 = None

# MQTT message callback
def on_message(client, userdata, msg):
    global t0
    if t0 is None:
        t0 = time.time()
    t = time.time()
    dt = t - t0
    print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")
    writer.writerow([t, dt, msg.topic, msg.payload.decode()])
    csv_file.flush()

# Set up MQTT client
client = mqtt.Client()
client.on_message = on_message

try:
    print(f"Connecting to MQTT broker at {args.host} and subscribing to topics {args.topics}")
    client.connect(args.host, 1883, 60)
except Exception as e:
    print(f"Failed to connect to broker: {e}")
    sys.exit(1)

# Open CSV file for writing
csv_file = open(args.output, mode="w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["Time", "Delta T", "Topic", "Message"])  # CSV header

# Subscribe to topics and loop forever
for topic in args.topics:
    client.subscribe(topic)
try:
    client.loop_forever()
except Exception as e:
    print(f"Error: {e}")
finally:
    csv_file.close()