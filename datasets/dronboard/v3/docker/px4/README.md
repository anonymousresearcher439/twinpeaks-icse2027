# PX4 service
This docker image includes PX4, gazebo, and gzweb.
## How to run this service

First build the base image:
```bash
sudo apt install --yes build-essential
git clone git@github.com:DroneResponse/PX4-containers.git
cd PX4-containers/docker
make px4-dev-simulation-gzweb
```
This image includes gazebo, gzweb, and all the other dependencies need so that `make px4_sitl gazebo` works.

Next build this image:

```bash
docker build -t px4-simulation .
```

Last you can, start this container with:
```bash
docker run -it -p 8080:8080 --rm --gpus all -e HEADLESS=1 px4-simulation start_simulation.sh
```

After that you can access `gzweb` at http://127.0.0.1:8080