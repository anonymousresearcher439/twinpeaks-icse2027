#!/usr/bin/env bash
# Use this script to send a message to MQTT to help test the DroneResponse onboard pilot

# assert we got two args
if [ $# -lt 2 ]
then
    echo "Usage: send_mqtt_message.sh <topic> <message_file>"
    exit 1
fi

# Set MQTT_HOST to a default value if it is not already set
MQTT_HOST=${MQTT_HOST:-host.docker.internal}
TOPIC=$1
MESSAGE_FILE=$2
#MISSION_TOPIC=drone/Polkadot/task/new


cat $MESSAGE_FILE | mosquitto_pub -h $MQTT_HOST -t "$TOPIC" -s
cat $MESSAGE_FILE | mosquitto_pub -h mqtt_local -t "$TOPIC" -s

