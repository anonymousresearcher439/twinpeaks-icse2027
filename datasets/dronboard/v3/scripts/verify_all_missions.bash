#!/bin/bash
# This script verifies all missions in the ./missions directory
# get the path to the directory containing this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR

python ../src/dr_onboard_autonomy/verify_mission.py ../missions
