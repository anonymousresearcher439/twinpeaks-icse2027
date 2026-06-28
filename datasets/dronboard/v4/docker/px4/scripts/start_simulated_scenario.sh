#!/bin/bash

cat <<EOF | bash >/dev/null &

cd /usr/local/gzweb/
sleep 1
npm start

EOF

cd /home/user/Firmware/

# First check the positional args for a drone count before failing back to
# an environment variable and finally default to 10
STARTING_CONDITIONS="$1"

Tools/gazebo_sitl_scenario.sh -s "$STARTING_CONDITIONS"

# sleep 2
