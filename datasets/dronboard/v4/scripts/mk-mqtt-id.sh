#!/usr/bin/env bash
#
# This will create an MQTT ID so that our MQTT server logs are more readable.
# You pass in a general name for your connection and this will append a random
# string to the end.

# Set defaults

HELP_TEXT="
Use this script will create an MQTT ID.

MQTT lets you set a client ID when you connect to the server. When we do so, our
logs are more readable. But we can't have two clients with the same ID. So you can
use this script to create a unique ID. You pass in a general name for your connection
and this will append a random string to the end.

Usage: $(basename $0) [name]

Arguments:
    name        The general name used to create the MQTT ID
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

NAME="DEV-SCRIPT"
# Overwrite default name, if needed
if [ $# -ge 1 ]
then
    # get the last arg given to the script
    NAME="$1"
fi

# Create a random string
RANDOM_ID=$(cat /dev/urandom | tr -dc 'A-Z0-9' | fold -w 12 | head -n 1)

echo "$NAME-$RANDOM_ID"
exit 0