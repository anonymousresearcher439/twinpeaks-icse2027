# Tasks

This document explains how to use the tasks feature.

## Overview

To use tasks, send the drone a mission that includes the `RunTasks` state. For example, you can send the generic mission [`missions/smp-tasks.json`](../missions/smp-tasks.json) to multiple drones. This mission instructs the drone to take off, run tasks, fly home, and land.

## Task Workflow

1. **Send a mission** that includes the `RunTasks` state.
2. **Wait for** the [Drone Ready for Task](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#task-drone-ready-for-task) message. The drone sends this when it enters `RunTasks`.
3. **Send a task** using the [Drone Task](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#drone-task) message. A task is similar to a mission but with extra features (see below).
4. **Wait for the [Task Outcome](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#task-drone-task-outcome) message**. This is sent when the task finishes.  
   - If the task ends unexpectedly (e.g., "low_battery", "error", "failsafe", "abort", or "rtl"), you will get a `Task Outcome` message but not a `Drone Ready for Task` message.
5. **Wait for another [Drone Ready for Task](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#task-drone-ready-for-task)** message.
6. **Repeat steps 3–5** for each new task.
7. **Send the [End Task Loop Message](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#drone-end-task-loop)** when there are no more tasks. The drone will continue the original mission (e.g., fly home and land).

---

## Task Features

Tasks are like missions, but with these extras:

- **Task IDs:** Assign a `task_id` to each task. This allows you to cancel tasks later. **NOTE: EVERY TASK NEEDS A UNIQUE ID**. 
- **Custom Task Outcomes:** Define custom outcomes that end the task when specific state outcomes occur.

### Task IDs

When sending a task, you must specify a unique task ID. If dr-onboard sees the same task ID more than once it will ignore it.

To generate a unique task ID:

```python
import uuid

def task_id() -> str:
    """Generate a unique task ID (UUID v4)."""
    return str(uuid.uuid4())
```

### Custom Task Outcomes

You can define custom outcomes for tasks. Here's an example of how custom task outcomes work.

Suppose you want the drone to look at some `stare_position` until a timer ends or the drone sees somebody. In this case you might send a task with two custom outcomes:
- an outcome in case the drone sees someone.
- an outcome in case the timer ends and nobody was seen.

To specify a custom outcome, you create an ordinary transition where you include the following properties:

- `condition`: you specify the `condition` property. This property names a state outcome. This will be the trigger for your custom outcome.  If/When the state ends with this outcome, your task will end, with your custom outcome.
- `target`: You specify the `target` property. This is the name of your custom outcome. You get to make this up. It must be a string. This string becomes the task's outcome. The drone reports this outcome string in the [Task Outcome Messages](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#task-drone-task-outcome) and in the [Task Ready Messages](https://github.com/DroneResponse/Onboarding/blob/main/topics.md#task-drone-ready-for-task) messages.
- `is_task_outcome: true`: you mark the transition with `"is_task_outcome": true`. You must include this property in the object.



**Example:**

```json
{
  "task_id": "b4d7051b-b22f-4ba6-a48a-f027498c1520",
  "states": [
    {
      "name": "BriarHover",
      "class": "BriarHover",
      "args": {
        "hover_time": 5.0,
        "stare_position": {
          "latitude": 41.606879523876685,
          "longitude": -86.35603187231904,
          "relative_altitude": 1.0
        }
      },
      "transitions": [
        {
          "target": "TIMER DONE",
          "condition": "succeeded_hover",
          "is_task_outcome": true
        },
        {
          "target": "PERSON FOUND",
          "condition": "found",
          "is_task_outcome": true
        }
      ]
    }
  ]
}
```

- `TIMER DONE` is triggered by the `succeeded_hover` outcome.
- `PERSON FOUND` is triggered by the `found` outcome.

---

## Canceling a Task

To cancel a running task, send a message to:

```
drone/{uav_name}/task/cancel-current
```

**Message template:**

```json
{
  "uavid": "Polkadot",
  "task_id": "{TASK_ID}"
}
```

## Example Usage

To demonstrate Tasks, here's how you can send task related messages using the bash script: [scripts/send_task.sh](../scripts/send_task.sh)

To get started, you'll want to start a simulation. For example:
```bash
just simulate-local-px4
```

Once the drone is ready, you can send the mission: [`missions/smp-tasks.json`](../missions/smp-tasks.json)

```bash
just send-mission missions/smp-tasks.json Polkadot 10.222.0.1
```

From there, you'll want to wait for the Task Ready message.
You can use `mosquitto_sub` to watch for the message:

```bash
mosquitto_sub -h 10.222.0.1 -t drone/Polkadot/task/ready
```
This command will start printing the task ready messages once the drone starts sending them. 

Next you can send a task. you can use `send_task.sh` to do so.
Here's how to send the hover task:

First, set these environment variables for the UAV name and the MQTT server
```bash
export UAV_NAME=Polkadot
export MQTT_SERVER=10.222.0.1
```

Then in the same shell, run the `send_task.sh` script:
```bash
scripts/send_task.sh new missions/tasks/hover_task.json.txt
```
This script will send the specified task to the drone. The script looks at the environment variables to determin the MQTT topic and the MQTT server.

The task sent in the example should tell the drone to hover for 300 seconds.

To see the task outcome you can run:
```bash
mosquitto_sub -h 10.222.0.1 -t drone/Polkadot/task/outcome
```
The task outcome message will print when the drone is finished with the task.

If you wish to cancel the task, you can send a task cancel message by:
Setting these environment variables
```bash
export UAV_NAME=Polkadot
export MQTT_SERVER=10.222.0.1
```

Running `send_task.sh` with these arguments:
```bash
scripts/send_task.sh cancel missions/tasks/hover_task.json.txt
```

When you cancel the task, you should see a task outcome message, and a task ready message immediately. 

Once you see the task ready message you can send the drone a new task.

When you're done sending tasks you can send the `end-task-loop` message by:
Setting these environment variables, (again if needed)
```bash
export UAV_NAME=Polkadot
export MQTT_SERVER=10.222.0.1
```
Then running:
```bash
./scripts/send_task.sh end missions/tasks/end-task-loop.json.txt
```
