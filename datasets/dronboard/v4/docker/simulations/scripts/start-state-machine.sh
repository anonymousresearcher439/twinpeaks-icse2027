#!/bin/bash
set -x

# these inputs with defaults for ros params override any config file settings
fcu=${1:-ardupilot}
drone=${2:-Polkadot}
mqtt_local=${3:-mqtt_local}
mqtt=${4:-host.docker.internal}
gimbal=${5:-mavlink_gimbal_manager}

echo "FCU:" $fcu
echo "Drone name:" $drone
echo "MQTT local:" $mqtt_local
echo "MQTT GCS:" $mqtt
echo "Gimbal type:" $gimbal

source /catkin_ws/devel/setup.bash
rosrun dr_onboard_autonomy state_machine.py _fcu_type:=${fcu} _uav_name:=${drone} _mqtt_local_host:=${mqtt_local} _mqtt_host:=${mqtt} _gimbal_type:=${gimbal}