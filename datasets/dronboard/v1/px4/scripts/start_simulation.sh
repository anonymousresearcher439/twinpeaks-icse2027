#!/bin/bash

cat <<EOF | bash >/dev/null &

cd /usr/local/gzweb/
sleep 1
npm start 

EOF

cd /home/user/Firmware/
make px4_sitl gazebo_typhoon_h480

# sleep 2
