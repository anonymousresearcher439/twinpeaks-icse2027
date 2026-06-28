#!/bin/bash
# Enable debugging
set -x
# Exit script if any command fails
set -e

# This is what we need to do:
#
# cd ardupilot_gazebo
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install

# But this script is defensive and will try to function


## Default values
# Default path to ardupilot_gazebo
ARDUPILOT_GAZEBO_PATH="ardupilot_gazebo"

# If an argument is provided, override the default path
if [ "$#" -eq 1 ]; then
    ARDUPILOT_GAZEBO_PATH="$1"
elif [ "$#" -gt 1 ]; then
    echo "Usage: $0 [path_to_ardupilot_gazebo]"
    exit 1
fi
# Get the number of CPU cores
NUM_CPUS=$(nproc)

# Navigate to the specified ardupilot_gazebo directory
cd "$ARDUPILOT_GAZEBO_PATH"

# Create the build directory and navigate into it
mkdir -p build
cd build

# Run cmake and make using all available CPU cores
cmake ..
make -j$NUM_CPUS

# Install using make install
sudo make install

cd .. 

# source /usr/share/gazebo-9/setup.sh
# timeout 120 gzserver --verbose worlds/iris_arducopter_runway.world || echo ok
