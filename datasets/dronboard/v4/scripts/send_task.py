#!/usr/bin/env python3
from string import Template
from pathlib import Path
from typing import Union
import uuid
import paho.mqtt.publish as publish

def task_id() -> str:
    """Generate a unique task ID (UUID v4)."""
    return str(uuid.uuid4())


def load_task(path: Union[Path,str]) -> str:
    """Load a task from a file.

    TEMPLATE FORMAT
    ###############
    {
        "task_id": ${task_id},
        "states":[
            ...
        ]
    }
    """
    if isinstance(path, str):
        path = Path(path)

    return Template(path.read_text()).safe_substitute(task_id=task_id())

def main(drone: str, task_path: str, gcs="10.222.0.1") -> None:
    """Main function to load and print the task."""
    task_msg = load_task(task_path)
    print(f"Sending task to drone {drone}:")
    print(task_msg)
    topic = f"drone/{drone}/task/new"
    publish.single(topic, task_msg, hostname=gcs, port=1883)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Send a task to a drone.")
    parser.add_argument("drone", type=str, help="Drone identifier")
    parser.add_argument("task_path", type=str, help="Path to the task file")
    parser.add_argument("--gcs", type=str, default="10.222.0.1", help="Ground Control Station IP address")
    args = parser.parse_args()
    main(args.drone, args.task_path, args.gcs)   
