#!/usr/bin/env bash
# Use this script to send a message to MQTT to help test the DroneResponse onboard pilot

MISSION_TOPIC=drone/unknown_uav/mission-spec
if [ $# -eq 0 ]
then
    MESSAGE_FILE=mission-spec.json
    #MESSAGE_FILE=mission.json
    #MESSAGE_FILE=mission_vision.json
else
    MESSAGE_FILE="$1"
fi
cat $MESSAGE_FILE | mosquitto_pub -h 127.0.0.1 -t "$MISSION_TOPIC" -s
