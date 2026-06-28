#!/usr/bin/env bash
#
# Note: You must have mosquitto_pub installed.
# On Ubuntu you can get it by installing:
#   sudo apt-get install mosquitto-clients
#

# Set defaults
if [ -z "$MQTT_SERVER" ]
then
    MQTT_SERVER="host.docker.internal"
fi

if [ -z "$UAV_NAME" ]
then
    UAV_NAME="Polkadot"
fi

# Check if we're trying to specify the UAV_NAME as an argument
if [ "$#" -eq 4 ] && ( [ "$3" == "--name" ] || [ "$3" == "--uav_name" ] )
then
    UAV_NAME="$4"
fi

MQTT_TOPIC_NEW="drone/$UAV_NAME/task/new"
MQTT_TOPIC_CANCEL="drone/$UAV_NAME/task/cancel-current"
MQTT_TOPIC_END="drone/$UAV_NAME/task/end-task-loop"


HELP_TEXT="
Use this script to send a task command to a drone.

Usage: $(basename $0) [command] [message_file]

Or

Usage: $(basename $0) [command] [message_file] --name [uav_name]

Arguments:
    command         The command to send to the drone. This must be one of the following: 'new', 'cancel', or 'end'
    message_file    The file containing the message to send
    uav_name        The name of the UAV. This is used to set the MQTT Topic. This is optional. If not provided, we use an environment variable or a default value.

Commands:
The 'new' command sends a new task to the drone.

The 'cancel' command cancels the task that the drone is currently executing. This command requires that you provide the task file that matches the current task.

The end command sends the 'end-task-loop' command. This tells the drone to stop executing tasks and continue with its mission.

Variables:
    UAV_NAME        The name of the drone. This is used to set the MQTT Topic
    MQTT_SERVER     The MQTT server to connect to

By default:
 - UAV_NAME='$UAV_NAME'
 - MQTT_SERVER='$MQTT_SERVER'

To change the UAV_NAME, set the UAV_NAME environment variable.
To change the MQTT server, set the MQTT_SERVER environment variable.

Examples

Example 1:
To specify the UAV_NAME and MQTT_SERVER inlined:

    UAV_NAME=$UAV_NAME MQTT_SERVER=$MQTT_SERVER $(basename $0) [command] [message_file]

Example 2:
To specify the UAV_NAME and MQTT_SERVER as environment variables:

    export UAV_NAME=$UAV_NAME
    export MQTT_SERVER=$MQTT_SERVER
    $(basename $0) [command] [message_file]

Example 3:
To send a new task command to the default UAV_NAME using the default MQTT_SERVER:

    $(basename $0) new task.json

Example 4:
To send an end-task-loop command to the default UAV_NAME using the default MQTT_SERVER:

    $(basename $0) end end-task-loop.json

Example 5:
To send an end-task-loop command to the Purple drone using the default MQTT_SERVER:

    $(basename $0) end end-task-loop.json --name Purple

"

# First check if we need to print the help text
# we should do so if we didn't get exactly 2 arguments or 4 arguments
if [ "$#" -ne 2 ] && [ "$#" -ne 4 ]
then
    echo "$HELP_TEXT"
    exit 1
fi

# we should also print the help text if the first argument is --help or -h
if [ "$1" == '--help' ] || [ "$1" == '-h' ]
then
    echo "$HELP_TEXT"
    exit 1
fi

# Now we use a case statement to set the MQTT_TOPIC based on the command
# I want to make sure $1 is lowercase before I compare it
CMD="$(echo $1 | tr '[:upper:]' '[:lower:]')"

case "$1" in
    new)
        MQTT_TOPIC=$MQTT_TOPIC_NEW
        ;;
    cancel)
        MQTT_TOPIC=$MQTT_TOPIC_CANCEL
        ;;
    end)
        MQTT_TOPIC=$MQTT_TOPIC_END
        ;;
    *)
        echo "Invalid command: $1"
        echo "The command must be either 'new' or 'end'"
        echo "For help, run: $0 -h"
        exit 1
        ;;
esac

MESSAGE_FILE=$2
# make sure the message file exists
if [ ! -f "$MESSAGE_FILE" ]
then
    echo "Message file not found: $MESSAGE_FILE"
    echo "For help, run: $0 -h"
    exit 1
fi

echo "Sending Task Command: $CMD"
echo "    MQTT Server:    $MQTT_SERVER"
echo "    MQTT Topic:     $MQTT_TOPIC"
echo "    Message File:   $MESSAGE_FILE"

# Add a random string to the end of the ID to prevent ID collisions
# when sending multiple missions in quick succession
RANDOM_ID=$(cat /dev/urandom | tr -dc 'A-Z0-9' | fold -w 12 | head -n 1)
cat "$MESSAGE_FILE" | mosquitto_pub -h "$MQTT_SERVER" -t "$MQTT_TOPIC" -s --id "SEND_TASK-SCRIPT-$RANDOM_ID"

echo "Done."