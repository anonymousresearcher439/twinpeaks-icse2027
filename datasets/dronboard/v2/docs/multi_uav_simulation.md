# Multiple UAVs in Gazebo

## Prerequisites
See the Host machine prerequisites section in the [running_a_simulation_locally](./running_a_simulation_locally.md) doc.

## Configuration
With the current docker compose definitions, up to 10 drones can be spawned in the containerized gazebo simulation. This is because there are 10 mavros instances available. The number of drones spawned is set via the `NUMBER_OF_DRONES` environment variable for the `px4` service in the `docker-compose.yml`.

## Running the simulation
```bash
NUM_DRONES=5 just simulate-multi $NUM_DRONES
```

The above command will launch `roscore` and `px4` services and then template out using ordinal naming (`drone_0..drone_n`, `mavros_0..mavros_n`, `mqtt_local_0..mqtt_local_n`) the number of drone instances passed to the command.

One of the critical components to this is setting the `ROS_NAMESPACE=uav{n}` on the mavros and drone services. Without this they are unable to determine how to communicate with one another. This can also be set at the `state_machine.py` entry point with `__ns:=uav{n}`.

You can send a mission to each drone using the following and replacing `{n}` with the drone instance you want to target.

```bash
just send-mission peppermint/big-circle drone_{n}
```

NOTE: as with the single drone simulation, any necessary additional services such as the [air leasing service](https://github.com/DroneResponse/microservice-air-lease/blob/main/README.md) must be running

To shut down the simulation simply use `Ctrl+C` and wait for all of the services to stop.

The simulation will leave multiple stopped containers if this is undesirable you can perform a clean up by running to delete all the old containers. Note that this NOT remove images and consequently the build assets.

```bash
just purge-containers
```

## More docs

- [Gazebo/px4](https://docs.px4.io/v1.12/en/simulation/multi_vehicle_simulation_gazebo.html)
