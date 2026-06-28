#!/bin/bash
#
# Use this script to test dr_onboard's ability to geo-location an object found by the camera
# this send a message to request the location of an object.
# dr_onboard will respond with a message containing the location of the object

MQTT_SERVER=${1:-${MQTT_SERVER:-mqtt_local}}
MQTT_PORT=${2:-${MQTT_PORT:-1883}}

TOPIC="dr-onboard/req-location-from-frame-data"
TIME_STAMP=$(($(date +%s%N) / 1000000))
MESSAGE=$(cat <<EOF
{
  "timestamp": $TIME_STAMP,
  "x": 960,
  "y": 810,
  "x_res": 1920,
  "y_res": 1080
}
EOF
)

echo "Sending message to $MQTT_SERVER:$MQTT_PORT on topic $TOPIC"
echo $MESSAGE | mosquitto_pub -h $MQTT_SERVER -p $MQTT_PORT -t 'dr-onboard/req-location-from-frame-data' -s
