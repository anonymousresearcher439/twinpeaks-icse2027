#!/bin/bash

cat <<EOF | bash >/dev/null &

cd /usr/local/gzweb/
sleep 1
npm start

EOF

cd /home/user/Firmware/

# First check the positional args for a drone count before failing back to
# an environment variable and finally default to 10
num_drones=${1:-${NUMBER_OF_DRONES:-10}}

Tools/gazebo_sitl_multiple_run.sh -m typhoon_h480 -n $num_drones

# sleep 2
