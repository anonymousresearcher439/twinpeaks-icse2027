#!/usr/bin/env python3
import sys
import time
import heapq
import signal
import threading
import json
import queue
from pathlib import Path
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

# ---- Known drones -----------------------------------------------------------
DRONES = ["Red","Crimson", "MidnightBlue", "Navy", "Lime", "Red"]

def get_entry_mission_file(uavid: str) -> Path:
    filename = f"{uavid.lower()}-on-entry.json"
    return Path("nasa") / filename

# ---- Timer utilities --------------------------------------------------------
START_TIME = time.monotonic()

def t_seconds() -> int:
    """Elapsed time in whole seconds since program start."""
    return int(time.monotonic() - START_TIME)

# ---- Message queue for incoming MQTT messages -------------------------------
message_queue = queue.Queue()

# ---- MQTT Subscriber callbacks ----------------------------------------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[T+{t_seconds():>5}s] MQTT connected. Subscribing to 'tasks ready'...")
        #client.subscribe("update_drone") # previously 'update_drone' messages.
        for drone in DRONES:
            topic = f"drone/{drone}/task/ready"
            print(f"[T+{t_seconds():>5}s] Subscribing to topic: {topic}")
            client.subscribe(topic)
    else:
        print(f"[T+{t_seconds():>5}s] MQTT connection failed with code {rc}")

# def on_message(client, userdata, msg):
#     try:
#         payload = msg.payload.decode("utf-8")
#         data = json.loads(payload)
#         message_queue.put(data)
#     except Exception as e:
#         print(f"[T+{t_seconds():>5}s] Failed to process MQTT message: {e}")
def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        # Extract drone ID from topic if not present in payload
        topic_parts = msg.topic.split("/")
        if "uavid" not in data and len(topic_parts) >= 2:
            data["uavid"] = topic_parts[1]

        message_queue.put((msg.topic, data))
    except Exception as e:
        print(f"[T+{t_seconds():>5}s] Failed to process MQTT message: {e}")


def start_mqtt_subscriber():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("127.0.0.1", 1883, 60)
    threading.Thread(target=client.loop_forever, daemon=True).start()
    return client

# ---- Minimal second-based scheduler ----------------------------------------
class Scheduler:
    """Schedule callables to run after N seconds from program start."""
    def __init__(self):
        self._q = []
        self._cv = threading.Condition()
        self._stop = False

    def after(self, seconds: int, func, *args, **kwargs):
        run_at = START_TIME + seconds
        with self._cv:
            heapq.heappush(self._q, (run_at, func, args, kwargs))
            self._cv.notify()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop:
            with self._cv:
                if not self._q:
                    self._cv.wait(timeout=0.5)
                    continue
                run_at, func, args, kwargs = self._q[0]
                now = time.monotonic()
                if now < run_at:
                    self._cv.wait(timeout=run_at - now)
                    continue
                heapq.heappop(self._q)
            try:
                func(*args, **kwargs)
            except Exception as e:
                print(f"[T+{t_seconds():>5}s] Event error: {e}")

    def stop(self):
        self._stop = True
        with self._cv:
            self._cv.notify_all()

# ---- Mission publishing -----------------------------------------------------
def publish_mission(drone_name: str, mission_file: Path):
    topic = f"drone/{drone_name}/mission-spec"
    mission = Path(mission_file).read_text()
    publish.single(topic, mission, hostname="127.0.0.1", port=1883)

    ts = f"[T+{t_seconds():>5}s]"
    print(f"{ts} Drone name: {drone_name}")
    print(f"{ts} Mission file: {mission_file}")
    print(f"{ts} sending mission {mission_file} to drone {drone_name}")
    print(f"{ts} Publishing to topic: {topic}")

# ---- Main -------------------------------------------------------------------
def main():
    STARTING_MISSIONS = {
        "Crimson": "./nasa/red-mission-1.json",
        "Red": "./nasa/red-mission-2.json",
        "MidnightBlue": "./nasa/blue-mission-1.json",
        "Navy": "./nasa/blue-mission-2.json",
    }

    scheduler = Scheduler()
    scheduler.start()

    # Start MQTT subscriber
    start_mqtt_subscriber()

    def handle_sigint(signum, frame):
        print(f"[T+{t_seconds():>5}s] Shutting down...")
        scheduler.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print(f"[T+{t_seconds():>5}s] Program started. Known drones: {', '.join(DRONES)}")

    if len(sys.argv) == 3:
        # Legacy mode: send mission to one drone
        drone_name = sys.argv[1]
        mission_file = Path(sys.argv[2])

        if drone_name not in DRONES:
            print(f"Unknown drone '{drone_name}'. Choose one of: {', '.join(DRONES)}")
            sys.exit(1)

        scheduler.after(0, publish_mission, drone_name, mission_file)

    elif len(sys.argv) == 1:
        # New mode: batch launch from STARTING_MISSIONS
        for i, (drone_name, mission_path) in enumerate(STARTING_MISSIONS.items()):
            if drone_name not in DRONES:
                print(f"[WARN] Skipping unknown drone: {drone_name}")
                continue
            scheduler.after(i * 2, publish_mission, drone_name, Path(mission_path))
     
        # Schedule SpringGreen separately at 120 seconds
        scheduler.after(
            20,
            publish_mission,
            "Lime",
            "nasa/lime-start.json"
        )

    else:
        print(f"Usage:\n  {sys.argv[0]}                  # to launch all predefined missions\n"
              f"  {sys.argv[0]} <drone> <file>   # to launch a specific mission")
        sys.exit(2)

    # ---- Main Loop ----
    try:
        while True:
            time.sleep(1)

            while not message_queue.empty():
                topic, msg = message_queue.get()
                uavid = msg.get("uavid")

                ts = f"[T+{t_seconds():>5}s]"

                if topic.endswith("/task/ready"):
                    print(f"{ts} [READY] Drone:{uavid} reported ready, sending entry mission...")

                    if uavid not in DRONES:
                        print(f"{ts} [WARN] Unknown drone {uavid} in task/ready message.")
                    else:
                        mission_file = get_entry_mission_file(uavid)
                        if mission_file.exists():
                            publish_mission(uavid, mission_file)
                        else:
                            print(f"{ts} [ERROR] Mission file not found: {mission_file}")
                else:
                    # Default behavior for status-style messages
                    status = msg.get("status", {})
                    st = status.get("status")
                    mode = status.get("mode")
                    pilot = status.get("onboard_pilot")
                    print(f"{ts} [UPDATE] Drone:{uavid}, Status:{st}, Mode:{mode}, Pilot:{pilot}")

                message_queue.task_done()

    except KeyboardInterrupt:
        handle_sigint(None, None)

if __name__ == "__main__":
    main()
