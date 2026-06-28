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

## Run the Microservice-airlease

The latest branches need the microservice-airlease to be running before the drones can fly.

Git clone it from [here](https://github.com/DroneResponse/microservice-air-lease).

Then cd into the cloned directory.

Change the MQTT broker address in settings.json file to match your machine's or use 127.0.0.1 (localhost).

Then run:

```bash
python3 airspace_lease.py
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

## VS Code Dev Container

### Setup
This project is set up to run in a VS code development container. This requires:

- Docker and docker-compose. You can install Docker Desktop, or use the [docker convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script).
- The VS Code [Remote Development Extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

To get started:

1. Open this project in VS Code
2. Press `ctrl+shift+p` or `cmd+shift+p` and enter: `Remote-Containers: Open Folder in Container`. This will build all the necessary containers, start the background services, and open the project from within the `dev` container.

### Usage

You can run unit tests within VS Code. You can open a test file and launch the run configuration called `Python: Test Current File` (found in the `Run` view). Also, you can run the unit tests using the built-in Terminal. For example:

```bash
# for all tests:
pytest test

# for a specific test module
pytest test/test_Heartbeat.py
```

`roscore`, `mavros`, `mqtt` and `mqtt_local` are started in the background. These other services are available to you from within the container. Furthermore, you can run ros CLI commands, `mosquitto_pub`, and `mosquitto_sub`.

For example, to check if mavros is connected to PX4, open the built-in terminal and look at the output from:

```bash
rostopic echo mavros/state
```

If you see a message containing `connected: True` then mavros is connected. If PX4 isn't connected, then:

- make sure PX4 is running
- make sure PX4 is configured to send mavlink over UDP to the mavros container. The mavros container is configured to listen on the host at `0.0.0.0:14540`. So you can configure PX4 to send mavlink to your host's IP address. 

If needed, configure a simulated instance of PX4 to send mavlink to the container, consider applying this patch in `PX4-Autopilot/Tools/sitl_gazebo`:

```patch
diff --git a/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink b/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
index 88aa1fe722..c690230f97 100644
--- a/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
+++ b/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
@@ -23,7 +23,7 @@ mavlink stream -r 20 -s RC_CHANNELS -u $udp_gcs_port_local
 mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local
 
 # API/Offboard link
-mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote
+mavlink start -p -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote
 
 # Onboard link to camera
 mavlink start -x -u $udp_onboard_payload_port_local -r 4000 -f -m onboard -o $udp_onboard_payload_port_remote
```

### Running `state_machine.py`
To run the project, open an integrated terminal window and enter:

```bash
python src/dr_onboard_autonomy/state_machine.py _uav_name:=Polkadot _mqtt_host:=mqtt _local_mqtt_host:=mqtt_local
```

Then to send a mission, open a second terminal window and enter:
```
MQTT_SERVER=mqtt ./send_mission.sh Polkadot missions/takeoff-test.json
```

### Debugging

To debug `state_machine.py` uncomment the relevant lines at the bottom of `state_machine.py`

```python
if __name__ == "__main__":
    import debugpy
    # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
    debugpy.listen(5678)
    print("Waiting for debugger attach")
    debugpy.wait_for_client()
    main()
```

1. Add breakpoints
1. Open the `Run and Debug` View (`Cmd+Shift+D` or `Ctrl+Shift+D`)
1. Launch the `Python: Remote Attach` _Run Configuration_ (found in the `Run and Debug` view)

To debug a unit test
1. Add breakpoints
1. Open the `Run and Debug` View (`Cmd+Shift+D` or `Ctrl+Shift+D`)
1. Launch the `Python: Test Current File` _Run Configuration_ (found in the `Run and Debug` view)


### ~~Pre-commit~~

> TODO Update this section

Note our precommit instructions are out of date. 

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

Please download the `1.13` release of PX4 using the following command:

```bash
git clone --recurse-submodules --branch v1.13.0 https://github.com/PX4/PX4-Autopilot.git
```

In Gazebo, if the drone does not takeoff due to RC failsafe, set this parameter in PX4 console:

```bash
param set COM_RCL_EXCEPT 4
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
