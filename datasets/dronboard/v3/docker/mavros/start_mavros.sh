#!/bin/bash

FCU_URL=${1}

/usr/bin/sleep 10

function start_streams {
    sleep 5
    rosrun mavros mavsys rate --all 10
}

# start_streams &
stdbuf -o L roslaunch --wait mavros apm.launch fcu_url:=$FCU_URL