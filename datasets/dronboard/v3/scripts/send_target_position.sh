#!/usr/bin/env bash

# Display help information
display_help() {
    echo "Overview: Use this script to send the target position over MQTT to the drones"
    echo
    echo "Usage: $0 [MQTT_HOST] [MESSAGE_FILE]"
    echo
    echo "Arguments:"
    echo "  MQTT_HOST     MQTT host to use. Defaults to the MQTT_HOST environment variable, else to '$DEFAULT_MQTT_HOST'."
    echo "  MESSAGE_FILE  File containing the message. Defaults to the MESSAGE_FILE environment variable, else to '$DEFAULT_MESSAGE_FILE'."
    echo
    echo "Options:"
    echo "  -h, --help    Display this help message."
    echo
    echo This script will send the specified message file to '"target_position"' MQTT topic.
    echo The MQTT host can be specified as an argument or in the MQTT_HOST environment variable.
    echo The message file can be specified as an argument or in the MESSAGE_FILE environment variable.
    echo The message file should be a JSON file containing the target position with the following structure:
    echo
    echo '    { "latitude": 0.0, "longitude": 0.0, "altitude_amsl": 230.0 }'
    echo
    echo
    echo Example 1:
    echo '    ./send_target_position.sh mqtt message.json'
    echo
    echo Example 2:
    echo '    MQTT_HOST=mqtt MESSAGE_FILE=message.json ./send_target_position.sh'
    echo

    echo the core of this script is the following command:
    echo
    echo '    cat "$MESSAGE_FILE" | mosquitto_pub -h "$MQTT_HOST" -t "$TOPIC" -s'
    echo
}

TOPIC="target_position"

# Default values
DEFAULT_MQTT_HOST="10.223.0.1"
DEFAULT_MESSAGE_FILE="$(dirname "$0")/../messages/peppermint-rd-target-position.json"

# Check for help argument
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    display_help
    exit 0
fi

# Check for command line args or environment variables or use default
MQTT_HOST=${1:-${MQTT_HOST:-$DEFAULT_MQTT_HOST}}
MESSAGE_FILE=${2:-${MESSAGE_FILE:-$DEFAULT_MESSAGE_FILE}}

# Publish using mosquitto_pub
echo "Sending target position..."
echo "    MQTT Host: $MQTT_HOST"
echo "    Topic: $TOPIC"
echo "    Message file: $MESSAGE_FILE"
echo
echo ----------------------------------Message Data----------------------------------
cat "$MESSAGE_FILE"
echo -e '\n--------------------------------------------------------------------------------'
echo
cat "$MESSAGE_FILE" | mosquitto_pub -h "$MQTT_HOST" -t "$TOPIC" -s
echo "Done."
