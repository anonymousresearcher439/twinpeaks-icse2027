# Onboard Pilot
Documentation will be uploaded soon

# Install

Make sure you have a ROS noetic dev environment set up. [Ubuntu setup instructions](http://wiki.ros.org/noetic/Installation/Ubuntu). Please look at the **New Instructions** to install the correct version of dependencies before following the basic instructions.

# Install dependencies

```bash
sudo apt install --yes ros-noetic-mavros ros-noetic-mavros-extras

cd /tmp
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

cd ~/catkin_ws/src/dr_onboard_autonomy
python3 -m pip install -r requirements.txt
```

# Create a catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
ln -s /path/to/this/repo/DR-OnboardAutonomy ~/catkin_ws/src/dr_onboard_autonomy
cd ~/catkin_ws/
cakin_make
catkin_init_workspace
source devel/setup.bash
```




## Usage

Launch the MAVROS node:
```bash
roslaunch dr_onboard_autonomy launch_mavros_node.launch
```

### Test that MAVROS launched

```bash
rostopic echo /birdy0/mavros/state
```

You should see state messages in the terminal.

### Run the state machine

Start the state machine:

```bash
rosrun dr_onboard_autonomy state_machine.py
```

You should see the drone takeoff, hover, and land.
If you're using a simulated drone and it doesn't arm, you might need to change some PX4 parameters.

```
param set COM_ARM_MAG_STR 0
```

# Developer Setup

### Pre-commit

`pre-commit` is a command-line tool to manage git hooks. You can find more about
it at [pre-commit.com](https://pre-commit.com/). The following is taken from
the `pre-commit` docs.

Install `pre-commit`

```bash
python3 -m pip install pre-commit
```

Set up the git hooks

```bash
pre-commit install
```

If needed run the pre-commit hooks on all files
```
pre-commit run --all-files
```

# Install PX4

Please download the latest stable release of PX4 (works with our project) using the following command:

```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

# New Instructions:

The above instructions for running the code are still good for running old branches such as offboard-control or preflight, but new branches such as dev or stable require additional steps to be followed because in the new branches, we are sending the mission-spec(states, transitions, waypoints, etc.) from a json file. The onboard pilot is using a library that can be installed with the following command:

```bash
pip3 install http://docs.q3w.co/droneresponse_mathtools-0.2.1-py2.py3-none-any.whl
```
Also the nvector version needs to be correct:

```bash
pip3 install nvector==0.7.6
```
Code in the dev and stable branches can be run using two different launch files in two different ways. The simplest way to run is using the following commands:

```bash
roslaunch dr_onboard_autonomy launch_mavros_node.launch
```

```bash
rosrun dr_onboard_autonomy state_machine.py
```

The json file needs to be sent from the UI, in case the UI is not available, the json file can be sent using a bash script, there is a bash script named mission.sh and there is a json file named mission-spec.json in the dev and stable branches. Putting both the files in the same directory, we can run this:

./mission.sh

In the line 4 of the mission.sh file, the uav name in the topic name must match the uav name in state_machine.py (line 208 for now). For instance, if the UAV name is **unknown_uav**, then the topic name in line 4 should be **drone/unknown_uav/mission-spec**

Instead of creating a symbolic link to the DR-OnboardAutonomy directory inside the catkin_ws (as described in lines 22-27 in this readMe file), we can also rename it to dr_onboard_autonomy and copy it directly to the catkin_ws/src/, then we can treat dr_onboard_autonomy as a ROS package. The disadvantage is then we cannot git checkout between branches directly but need to copy it over everytime there is a change in the code or in the branches but if creating symbolic link does not work, this is an alternative option.

We do not want the drone's origin location to be too far from where we want the drone to fly to. So, it is necessary to setup the home location of the drone in PX4. We can setup the following home location which is closer to the drone's destination flight location in the json file:

```bash
export PX4_HOME_LAT=41.70573086523541
export PX4_HOME_LON=-86.24421841999177
export PX4_HOME_ALT=0
```
The home location needs to be setup in the terminal from which we launch PX4.

There is another way of running the code in which we can send the PX4 home location, drone's name, MQTT broker's IP address as command line arguments. We can also decide if we want to run PX4 in Headless mode which means without GUI. To run the code, that way, we can clone the PX4 directory into the catkin_ws/src/ and rename it to px4.

Then we want to build the px4 directory using these commands:

```bash
cd px4/
./Tools/setup/ubuntu.sh
make px4_sitl_default
make px4_sitl jmavsim
make px4_sitl gazebo
```
Then we want to build our catkin_ws running **catkin_make**

We want to source it:

```bash
source devel/setup.bash
```
Then we can run:

```bash
roslaunch dr_onboard_autonomy onboard_pilot_mavros_px4_simulator.launch uav_name:="blue" latitude:="41.714757" longitude:="-86.242578" altitude:="221" mqtt_host:="sarec2.crc.nd.edu" HEADLESS:="1"
```
The launch file is also running the python script so only one command is needed to run the code. We still need to send the json file through the bash script as described above.


# Running YOLO on the Jetson:

All the python code, weights and cfg files for running YOLO with relevant instructions can be downloaded from [here](https://drive.google.com/drive/folders/1Qgp3Rqo7ogqaqtBwRBp_21vmacgqEa8b?usp=sharing).
