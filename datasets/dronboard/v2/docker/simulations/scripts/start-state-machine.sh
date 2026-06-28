#!/bin/bash
set -x

drone=${1:-Polkadot}
mqtt=${2:-host.docker.internal}
mqtt_local=${3:-mqtt_local}

echo $drone
echo $mqtt
echo $mqtt_local

source /catkin_ws/devel/setup.bash
rosrun dr_onboard_autonomy state_machine.py _uav_name:=${drone} _mqtt_host:=${mqtt} _local_mqtt_host:=${mqtt_local}