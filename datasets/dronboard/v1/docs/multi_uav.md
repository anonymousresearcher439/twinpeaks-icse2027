# Multiple UAVs in Gazebo
## Mac-OS (Monterey) - still testing on Linux
Assumes PX4 + gazebo running locally on host computer

### Modifications to docker compose
The docker compose setup is now optimized for fully containerized px4 + gazebo simulations. The 
containerized simulation only works for a single drone at the moment. The following setup still 
works for multi-drone simulations when px4 + gazebo are installed on the host computer. However,
the following services needs to be commented out in the `docker-compose.yml` file before running:
- px4
- vs-code

### Running the simulation
To spawn two UAVs in gazebo:
```bash
Tools/gazebo_sitl_multiple_run.sh -m iris -n 2
```
- make sure for each drone that the following PX4 parmeter is set accordingly, or you will run into a failsafe due to lack of an RC connection:
    - COM_RCL_EXCEPT=4

To start services needed for two uavs:
```bash
docker compose -f docker-compose.yml -f mac_multi_drone.yml up
```

To start two independent dev dr_onboard services each attached to a single mavros service via the ros namespace:
- first open a dev container for each dr_onboard service and source the ros catkin workspace environment:
```bash
docker compose run --no-deps dev
source /catkin_ws/devel/setup.bash
```
- then run the onboard state machine for each uav:
```bash
rosrun dr_onboard_autonomy state_machine.py _uav_name:=<unique drone name> _mqtt_host:=mqtt _local_mqtt_host:=mqtt_local __ns:=<unique name matching one mavros namespace>
```
- it is key that the name space specified with rosrun above matches that of one of the mavros instances
- it is also key that the drone names are unique for each instance as you want to be able to communicate to them separately over mqtt

The mac_multi_drone.yml file does a couple key things to allow for communciation to two separate drones:
1) starts two independent mavros instances each in their own ros name space using the ROS_NAMESPACE env variable. This allows each mavros service to communicate with a specific OnboardPilot node in the same name space
2) starts each mavros instance with a new udp port and incremented target system id as specified in the following documentation: https://docs.px4.io/v1.12/en/simulation/multi_vehicle_simulation_gazebo.html

To shut down the docker services for multiple uavs, make sure to run the following full command. 
```bash
docker compose -f docker-compose.yml -f mac_multi_drone.yml down
```
- simply running _docker compose down_ will result in an orphaned container since there is a new service specified in mac_multi_drone.yml: 
https://stackoverflow.com/questions/41494612/docker-compose-orphan-containers-when-overriding-services

