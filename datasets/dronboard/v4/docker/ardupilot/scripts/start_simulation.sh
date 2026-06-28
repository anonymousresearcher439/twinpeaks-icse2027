#!/bin/bash

# we need to setup the environment
# this is needed or ardupilot sitl won't start
# this is also needed so gazebo doesn't re-download the models. 
source ~/.profile

# a function to start gzserver
function start_gzserver() {
    # start gazebo
    cd /usr/local/ardupilot_gazebo/
    # this could take 30-60 seconds to start:
    # we need to download the gazebo assets from the internet
    # and it takes a while.... 
    # TODO download this stuff in the Dockerfile
    gzserver --verbose worlds/iris_arducopter_runway.world 
}

# a function to start gzweb
function start_gzweb() {
    # start gzweb
    cd /usr/local/gzweb/
    npm start
}

# start_arducopter_sitl
# Starts the ArduCopter SITL (Software In The Loop) simulation.
#
# Usage:
#   start_arducopter_sitl <args>
#
# Arguments:
#   <args>  The arguments to pass to the ArduCopter SITL.
#   these arguments are appended to the command below.
#  if you don't want to use QGC, you 
#   
# Example:
#   start_arducopter_sitl "udp:192.168.63.13:14550" "udp:mavros:15000"
#
function start_arducopter_sitl() {
    GCS=$1
    MAVROS=$2
    # start arducopter sitl
    cd /home/user/ardupilot/
    # append all the args to the command
    ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console \
        -l 41.606694311443235,-86.35561785419657,229.5,180 \
        $@
}

# start gzserver in the background
start_gzserver &
# start gzweb in the background
sleep 1
start_gzweb &
# start arducopter sitl. pass all the args to this script to the arducopter sitl
# function
start_arducopter_sitl $@

