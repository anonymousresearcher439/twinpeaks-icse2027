# Multiple UAVs in Gazebo

## Prerequisites
See the Host machine prerequisites section in the [running_a_simulation_locally](./running_a_simulation_locally.md) doc.

## Configuration
Up to 102 drones can be spawned with unique names. The `./docker/simulations/multi.yaml.template`, `./docker/simulations/multi/drone.yaml.template` along with the `./scripts/generate-drones.py` and `./scripts/drone-names.py` scripts create the necessary `yaml` files to run the multi-drone simulation. Today, the multi-drone simulation is only set up to run with `Px4`. 

## Running the simulation
```bash
NUM_DRONES=5 just simulate-multi $NUM_DRONES
```

The `./scripts/drone-names.py` script is used to uniquely name drones different colors. Drone names will be generated from the top down in the list. 

You can send a mission to each drone by referencing its auto-generated color based name:

```bash
just send-mission <path to mission file> <drone name (default=Polkadot)> <GCS host (default=localhost)>
```

An example sending a mission to a drone named "Red" when the mission is sent from the GCS computer:
```bash
just send-mission ./missions/peppermint/big-circle.json Red
```

NOTE: as with the single drone simulation, any necessary additional services such as the [air leasing service](https://github.com/DroneResponse/microservice-air-lease/blob/main/README.md) must be running

To shut down the simulation simply use `Ctrl+C` and wait for all of the services to stop.

The simulation may leave multiple stopped containers if this is undesirable you can perform a clean up by running the following to delete all the old containers. Note that this does NOT remove images and consequently the build assets.

```bash
just purge-containers
```

## More docs
- [Gazebo/px4](https://docs.px4.io/v1.12/en/simulation/multi_vehicle_simulation_gazebo.html)
