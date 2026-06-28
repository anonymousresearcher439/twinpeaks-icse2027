#!/usr/bin/env bash
#
# Note: You must have mosquitto_pub installed.
# On Ubuntu you can get it by installing:
#   sudo apt-get install mosquitto-clients
#






# Set defaults
if [ -z "$MQTT_SERVER" ]
then
    MQTT_SERVER="mqtt"
fi
UAV_NAME="default_uav"
MQTT_TOPIC="drone/$UAV_NAME/mission-spec"
MISSION_FILE="missions/mission-spec-test-briar-hover.json"

HELP_TEXT="
Use this script to send a mission to a drone.

Usage: $(basename $0) [uav_name] [mission_file]

Arguments:
    uav_name        The name of the drone. This is used to set the MQTT Topic
    mission_file    The file containing the mission

By default:
 - uav_name='$UAV_NAME'
 - mission_file='$MISSION_FILE'

When no arguments are given, the default value is used for both the uav_name and
the mission_file.

When 1 argument is given, it specifies the mission file. In this case, the
default value is used for the uav_name.

When 2 arguments are given, the first is the uav_name and the second specifies
the mission file.

To change the MQTT server, set the MQTT_SERVER variable.

For example:

    MQTT_SERVER=$MQTT_SERVER $(basename $0) [uav_name] [mission_file]
"

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
    MISSION_FILE="${@: -1}"
fi

# Overwrite default uav name, if needed
if [ $# -eq 2 ]
then
    UAV_NAME="$1"
fi
MQTT_TOPIC="drone/$UAV_NAME/mission-spec"

echo "Sending Mission"
echo "    MQTT Server:    $MQTT_SERVER"
echo "    MQTT Topic:     $MQTT_TOPIC"
echo "    Mission File:   $MISSION_FILE"

cat "$MISSION_FILE" | mosquitto_pub -h "$MQTT_SERVER" -t "$MQTT_TOPIC" -s

echo "Done."