# Running a simulation locally
## Host machine prerequisites
- docker engine (daemon)
- docker compose

On a Linux host, you will want to make sure to [install or upgrade](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) to the latest docker daemon and docker compose version. This setup was tested with:
- docker versions: 20.10.25, 24.0.6, 25.0.4
- docker compose versions: 1.29.2, 2.21.0, 2.24.7

It may work with earlier versions, but some features such as providing the `extra_hosts` service input in the docker compose file are only available in later versions. `extra_hosts` is used to map the address where px4 should send mavlink data to be received by QGroundControl.

## Configurations
### QGroundControl (QGC)
## px4
The `host.qgc` mapping under the px4 service is configured by default in the docker compose file to send QGC mavlink data to the host maching running the simulation containers. However, you may want to forward the QGC mavlink data to another IP on the network. If so, you can update the `extra_hosts` `host.qgc` configuration to point to your desired IP:

```yaml
px4:
  extra_hosts:
    - "host.qgc:<your.desired.ip.address>"
```

# ardupilot
The Ardupilot flight control unit exposes a tcp server that QGC can connect to. Port `5763` is exposed to the host from the ardupilot service. After the ardupilot service is up and running, QGC can connect via `5763` at `127.0.0.1`. This is set in QGroundControls `Application Setting -> Comm Links`.

## Steps to run
Running a simulation requires the Ground Control services to be running. In order to start those ensure you have cloned the [Ground-Control-Services](https://github.com/DroneResponse/Ground-Control-Services) repo.

### Ground Control Services

```bash
git clone git@github.com:DroneResponse/Ground-Control-Services.git
cd Ground-Control-Services
```

Once cloned start the `air-lease` service. This will start both itself and an `mqtt` broker available at `localhost:1883`.

```bash
docker compose up air-lease
```

or without logging in the terminal

```bash
docker compose up -d air-lease
```

### Local Simulation

Once the Ground Control Services are running you can kick off a local simulation using...

There are two options, you can simulate px4 or ardupilot.
Here's how:
```bash
just simulate-local-px4
```
or 
```bash
just simulate-local-ardupilot
```

This will start `roscore`, `mavros`, `px4` or `ardupilot`, and the `drone`. By default the drone will be named `Polkadot` and will attempt to attach to the ground control `mqtt` broker at `host.docker.internal:1883` so make sure you have that running.

Once the containers have all started running you can send a mission to the drone with...

```bash
just send-mission peppermint/big-circle Polkadot
```

**NOTE** The above command requires a local install of `mqtt` which is available on MacOS with `brew install mosquitto` or `sudo apt-get install mosquitto mosquitto-clients` on Debian.

Once the mission has completed a simple `Ctrl+C` will kill the containers.

**NOTE** The ardupilot simulator is started with a background `screen` session to keep mavproxy from exiting without access to a terminal window. Therefore, you will not see logs for the ardupilot service or be able to interact with mavproxy unless you re-attach to the screen session. You can re-attach to the screen session by doing the following after the ardupilot compose service is running:

```bash
docker exec -it <container name> bash
screen -r mavproxy
```

where `container name` is the container name for the `ardupilot` service. 
