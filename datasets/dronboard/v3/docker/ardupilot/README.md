# Ardupilot Simulator

To Build
```bash
docker build -t apm . 
```
To start, using QGC (or another external GCS)
```bash
docker run -it --rm -p 8080:8080 -p 5760:5760 -p 5762:5762 apm start_simulation.sh --no-mavproxy
```

To start without QGC (using mavproxy as the GCS)
```bash
docker run -it --rm -p 8080:8080 -p 5762:5762 apm start_simulation.sh
```
Note: when you run ardupilot with mavproxy, you can use the shell to interact with the drone. The docker process prompts you for commands. You can enter `help` to see the available commands.

To connect QGC, configure a Comm Link in **Application Settings**. Specifically with QGC acting as the TCP client, you connect to `localhost` port `5760`. 

To connect `mavros` use this flight controller URL
## How this works

This simulator has three main pieces:

1. the gazebo simulator
2. a simulated ardupilot flight controller (arducopter sitl)
3. gzweb

Optionally this runs mavproxy too. 

You can view the simulation by connecting to gzweb at http://localhost:8080

