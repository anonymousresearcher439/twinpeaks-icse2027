Instructions for running DR-Onboard on the Streamline Designs HX-10 drones. 

## Drone Config
Each drone now uses a config persisted to `/etc/dr/config.toml`. This config provides _dr-onboard_ drone specific information so it can self-configure.

The HX-10 config should look like:
```toml
# These are permanent configuration details that don't change run-to-run
name = "<unique name of the drone>"
system_id = 1 # this is the MAVLink system ID

[mqtt]
# The ground control MQTT server
host = "10.223.0.1"
port = 1883

[fcu]
# Flight control unit
# type: "ardupilot" | "px4"
type = "px4"

[gimbal]
# In python this is config['uav']['gimbal']['pitch'] or config.uav.gimbal.pitch for class based config
pitch = false
roll = false
yaw = false
# type: "gremsy" | "mavlink_gimbal_manager"
type = "mavlink_gimbal_manager"
```

The configurations should be the same across all HX-10 drones aside from the `name`. Update the config for each drone and place it here on the jetson: `/etc/dr/config.toml`. Make sure the file has read permissions for whichever user is running _dr-onboard_.

## Build
Use the version of `docker-compose` that comes included with the jetson to build:
```bash
sudo docker-compose -f docker/drone/jetson-hx10.yaml build
```

## DR-Onboard Starup
All docker related files are now stored under the top level `docker` directory.

If `just` is installed on the jetson, you can start up _dr-onboard_ and required onboard services with:
```bash
just start-drone jetson-hx10
```

Otherwise, you can start _dr-onboard_ and required onboard services with: 
```bash
sudo docker-compose -f ./docker/drone/jetson-hx10.yaml up
```
