<p align="center">
  <img src="https://github.com/DroneResponse/DR-OnboardAutonomy/blob/d15f03582348cd87a0f6bd1341e4ca255195c98e/docs/images/Drone_Response_Logo.png?raw=true" width="400" alt="logo"/>
</p>

---

[![⚙️ Unit Tests](https://github.com/DroneResponse/DR-OnboardAutonomy/actions/workflows/run-unit-tests.yml/badge.svg)](https://github.com/DroneResponse/DR-OnboardAutonomy/actions/workflows/run-unit-tests.yml)
[![🔧 Verify Missions](https://github.com/DroneResponse/DR-OnboardAutonomy/actions/workflows/verify-missions.yml/badge.svg)](https://github.com/DroneResponse/DR-OnboardAutonomy/actions/workflows/verify-missions.yml)

# Table of Contents

- [Developer Environment Setup](#developer-environment-setup)
- [Run a Simulation Locally](docs/running_a_simulation_locally.md)
- [Multiple UAV Simulation](docs/multi_uav_simulation.md)
- [Serial to Jetson Simulation](docs/serial_to_jetson_simulation.md)
- [Tests](#tests)


# Developer Environment Setup

## Visual Studio Dev Container

### Setup
This project is set up to run in a VS Code development container. This requires:

- Docker and docker-compose. You can install Docker Desktop or Docker Engine via [one of Docker's recommended installation methods](https://docs.docker.com/engine/install/ubuntu/#installation-methods).
    - Note: For Windows Users, Docker Desktop is not recommended due to resource and stability issues. Instead, install WSL2 Ubuntu and install Docker and Docker Compose from the CLI. The VS Code Remote Development Extension is then utilized.
- [Visual Studio Code](https://code.visualstudio.com/download)
- [VS Code Remote Development Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

To get started:

1. Clone the relevant repositories with GIT
    - [DR-OnboardAutonomy](https://github.com/DroneResponse/DR-OnboardAutonomy)
    - [Ground Control Services](https://github.com/DroneResponse/Ground-Control-Services)
    - [AirSim](https://github.com/DroneResponse/AirSim) (Optional)
    - [DR-Hardware](https://github.com/DroneResponse/DR-Hardware) (Optional)
2. Open this project in VS Code
3. Press `ctrl+shift+p` or `cmd+shift+p` and enter: `Remote-Containers: Open Folder in Container`. This will build all the necessary containers, start the background services, and open the project from within the `dev` container.

### Usage
#### Tests
You can run unit tests within VS Code. You can open a test file and launch the run configuration called `Test: Run Tests in Current File`. Also, you can run the unit tests using the built-in Terminal. For example:

```bash
# for all tests:
pytest test

# for a specific test module
pytest test/test_Heartbeat.py
```

You can also run the unit tests without entering the dev container by entering `docker compose run unit_tests` while in the DR-OnboardAutonomy repository on your host. 

#### Services
`roscore`, `mavros`, `mqtt` and `mqtt_local` are started in the background. These other services are available to you from within the container and are accessible via a hostname that matches their service name. For example, you could run `ping mqtt`. Furthermore, you can run ros CLI commands, `mosquitto_pub`, and `mosquitto_sub`.

For example, to check if mavros is connected to PX4, open the built-in terminal and look at the output from:

```bash
rostopic echo mavros/state
```

If you see a message containing `connected: True` then mavros is connected.

#### Debug
The quickest way to debug with minimal setup is to insert the line `import pdb; pdb.set_trace()` wherever you would like to enter into the debugger.

Keep in mind, the debugger stops the main thread of execution. If you use the debugger while running unit tests, the behavior will be as expected. If you use the debugger while running a simulation, expect the drone to quickly enter a failsafe mode - and likely land. This occurs because the flight controller expects setpoints from DR-Onboard within a couple second maximum threshold while in offboard mode. Pausing execution for the debugger causes DR-Onboard to stop sending the required setpoints. 


# Tests
## Unit Tests
All unit tests belong in the `test` directory and modules must be prefixed with `test_`

To run all unit tests:
```bash
python -m unittests
```

## Integration Tests
All integration tests belong in the `integration` directory and must be suffixed with `_integration`. Integration tests are meant to be run on hardware to check various communication channels and components. 

To run all integration tests:
```bash
python -m integration.run_integration_tests
```
