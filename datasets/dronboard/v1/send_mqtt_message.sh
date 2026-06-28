#!/usr/bin/env bash
# Use this script to send a message to MQTT to help test the DroneResponse onboard pilot

MISSION_TOPIC=drone/Polkadot/mission-spec
if [ $# -eq 0 ]
then
    MESSAGE_FILE=mission.json
else
    MESSAGE_FILE="$1"
fi
cat $MESSAGE_FILE | mosquitto_pub -h 10.223.0.1 -t "$MISSION_TOPIC" -s
