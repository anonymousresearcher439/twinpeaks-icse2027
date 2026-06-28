#!/usr/bin/env bash
#
# Note: You must have mosquitto_pub installed.
# On Ubuntu you can get it by installing:
#   sudo apt-get install mosquitto-clients
#

MQTT_SERVER="localhost"

HELP_TEXT="
Use this script to send a file as an MQTT message.

Usage: $(basename $0) [mqtt_topic] [message_file]

Arguments:
    mqtt_topic          The MQTT topic
    message_file        The file containing the message

When no arguments are given, a default value is used for both the mqtt_topic and the message_file.

When 1 argument is given, it specifies the message file. A default value is used for the mqtt_topic.

When 2 arguments are given, the first is the MQTT topic and the second is the message file.
"

# Set defaults
MQTT_TOPIC="drone/default_uav/mission-spec"
MESSAGE_FILE="mission.json"

# First check if we need to print the help text
if [ $# -eq 1 ]
then
    if [ "$1" == '--help' ] || [ "$1" == '-h' ]
    then
        echo "$HELP_TEXT"
        exit 1
    fi
fi

# Overwrite default message file, if needed
if [ $# -ge 1 ]
then
    # get the last arg given to the script
    MESSAGE_FILE="${@: -1}"
fi

# Overwrite default mqtt topic, if needed
if [ $# -eq 2 ]
then
    MQTT_TOPIC="$1"
fi

cat "$MESSAGE_FILE" | mosquitto_pub -h "$MQTT_SERVER" -t "$MQTT_TOPIC" -s 
