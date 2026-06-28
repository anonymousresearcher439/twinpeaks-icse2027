# Notes about `offboard` Control
This describes how `offboard` control is done in the `FlyWaypoints` state. The `FlyWaypoints` state is a work in progress. But the general ideas in this doc will remain relevant.

There are a few moving parts needed to get setpoint control working.

- We need to send setpoint messages at regular intervals. If we don't we will trigger the "connection is lost" fail-safe. The code referenced below sends them 10 times per second.
- We need to have already sent some setpoint messages before switching to `offboard` mode.

If we do everything right, then when we switch to `offboard` mode, the drone will accept our setpoint guidance.

The rest of this doc explains how `offboard` control is done in `FlyWaypoints`.

In the constructor of the state, we use a `RepeatTimer`. [Here is the line of code](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L26). The `RepeatTimer` puts a `setpoint` message in our queue every 0.1 seconds.

When the state receives a `setpoint` message, it needs to send another setpoint command. [Here is the function that runs when we receive a `setpoint` message.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L57-L60) The line `self.drone.send_setpoint(lla=current_waypoint)` is what actually sends the setpoint.

To trigger this function, we call the `self.handlers.add_handler` method in the constructor. This method, lets us specify a message type we care about, and a function we wish to run. When we receive a message of the given type, the function runs. [Here is the code.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L29)

That takes care of sending setpoints often enough. But we need to make sure that we have sent some setpoint messages before we switch to `offboard` mode. I think we can improve the way this is done in `FlyWaypoints`. But for the sake of explaining stuff, I will explain how it works.

The state counts the number of setpoint messages it has sent. [Here is the line that updates the count.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L60) Whenever the drone sends us a `state` message, we run the `trigger_offboard_mode` method. [Here is the method.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L62-L70) This method checks if we've sent some setpoint messages and if we're not in `offboard` mode, it changes to `offboard` mode. [Here is the line that causes the `trigger_offboard_mode` method to run when we receive a `state` message.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/src/states/FlyWaypoints.py#L31) I'm sure there is a better way to change the mode.

### When unit testing
`FlyWaypoints` needs unit tests but here's some notes about how it could be tested.

The tests should remove the `RepeatTimer` from the collection of `message_senders` before calling `execute`. If we don't then the repeat timer may put messages in the queue when we're not expecting. This could cause problems.

When constructing the state, we should use a mock `drone` object. [Here is an example](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/test/test_flight_states.py#L19). That line creates an object that looks like a drone. The test can then check that the state called methods on the `drone` like it should.

The state tries to reuse message senders. The constructor tries to get message senders from the `reusable_message_senders` argument. [We can mock this object like we do here.](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/de0a0ea564f374d9a539cf1d3b5bf54b89ffd7a7/test/test_flight_states.py#L22-L24)By using a mock message sender, we can control what gets put on the message_queue.
