#!/usr/bin/env bash
# Use this script to send a message to MQTT to help test the DroneResponse onboard pilot

# assert we got two args
if [ $# -lt 2 ]
then
    echo "Usage: send_mqtt_message.sh <topic> <message_file>"
    exit 1
fi

TOPIC=$1
MESSAGE_FILE=$2
#MISSION_TOPIC=drone/Polkadot/task/new


cat $MESSAGE_FILE | mosquitto_pub -h mqtt -t "$TOPIC" -s
