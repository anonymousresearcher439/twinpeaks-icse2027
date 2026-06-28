## Intro

At the project’s core is a `SMACH` state machine:

[http://wiki.ros.org/smach](http://wiki.ros.org/smach)

The link above explains how `SMACH` works and this document assumes you've read the [Documentation](http://wiki.ros.org/smach/Documentation) page and some of the [Tutorials](http://wiki.ros.org/smach/Tutorials).

The software builds and runs a `SMACH` state machine and our states extend `smach.State`. It happens to be the case that our `execute` methods block on a `queue.get()` call and we avoid blocking on any other long running functions. This queue is our state's connection to the outside world. While we're not blocking on `queue.get()` we might be unresponsive to important new developments.

Messages from the outside world get added into our [`Queue`](https://docs.python.org/3/library/queue.html#queue-objects) by other threads. The threads that actually put messages on the queue will be covered later. But these threads might get their data from ROS, MQTT, or some other source too.

By forcing all outside data into the queue, we can be reasonably sure that our code is safe from threading errors. This is important because all the data is coming to us from other computers or threads. From within a `State`, we only access data that we own and we own the messages we get once they arrive. Here is an example `State` that shows the concepts we've covered so far:

```python
class ExampleState(smach.State):
    def __init__(self):
        # more to come later
        self.message_queue = Queue()

    def execute(self, userdata):
        # more to come here
        while True:
            msg = self.message_queue.get()
            # do something with the messages here
```

We look at the messages we receive to decide when we're ready to transition to another state. For example, the state that arms the drone might transition once the flight controller reports that the motors are armed:

```python
class ArmExample(smach.State):
    def __init__(self):
        # more to come
        self.message_queue = Queue()

    def execute(self, userdata):
        # more to come
        while True:
            msg = self.message_queue.get()
            if is_drone_armed(msg):
                return "succeeded"

    def is_drone_armed(msg):
        # look at the message to determine if the drone is armed
        pass
```

And that is how our states work at a high-level. The rest of this document goes into the details and explains how the queues get fed, and the rest.

## How to get messages

Our states need messages to function. And something needs to `put` messages in the queue. In fact, our states need to receive messages from many other systems like ROS, MQTT, the operating system and more. The key abstraction, that's responsible for putting messages in our queue, is the message sender.

Here is what `MessageSenders` do:

1. They detect when something interesting happens.
1. They build a message that captures the details.
1. They `put` the message in the queue.

Lets build an example MessageSender.

### Example: An MessageSender for shutdown messages.

While the state is blocking on a call to `message_queue.get()`, the program cannot shutdown. To fix this, our state needs to receive a message when it’s time to shutdown. This example guides you through the creation of the `ShutdownMessageSender`. This sends a message to the state when the program wants to shut down.

Let’s start by deciding on the data we'd like to include in our message. This is the message we want to send when it’s time to shutdown:

```python
{"type": "shutdown", "data": True}
```

All messages should be of type `dict` and they must include a key/value pair for `type` and `data`.

The `type` entry tells the state what kind of message we’re sending. The state looks at this value to identify the subroutines it will run in response to our message.

The `data` entry could hold anything. It's value depends on the kind of message we’re sending. In this example the `data` value isn’t important. But in other messages, The `data` value is essential.  In a `position` message, for example, the data value holds the drone’s latitude, longitude, and altitude.

Now that we know what our message looks like, let’s sketch out a new MessageSender class:

```python
class ShutdownMessageSender:
    def __init__(self):
        self._to_active_state = None
        self.shutdown_message = {"type": "shutdown", "data": True}
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        if self.to_active_state is not None:
            self.to_active_state(self.shutdown_message)
```

This code captures the fundamental behavior we're looking for. When `rospy` wants to shutdown the process, it calls our `ShutdownMessageSender`'s `on_shutdown()` method. In turn, our method calls `self._to_active_state` with our shutdown message. This is a good start, but to make this all work, we need to link `self._to_active_state` to the active state's `message_queue.put` method.

Let's add a method called `start` that takes a state's `message_queue.put` reference as an argument:

```python
class ShutdownMessageSender:
    def __init__(self):
        self._to_active_state = None
        self.shutdown_message = {"type": "shutdown", "data": True}
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        if self.to_active_state is not None:
            self.to_active_state(self.shutdown_message)

    def start(self, to_active_state):
        self._to_active_state = to_active_state
```

The above code approaches the behavior that we want, but it's prone to a few race conditions. First, consider what would happen if `on_shutdown` was called before the active state had a chance to call `start`. In this case the shutdown message wouldn't be sent to the active state and our state wouldn't know it's time to shutdown. Let's fix this by detecting if `on_shutdown` was previously called:

```python
class ShutdownMessageSender:
    def __init__(self):
        self._to_active_state = None
        self._is_shuting_down = False
        self.shutdown_message = {"type": "shutdown", "data": True}
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        self._is_shuting_down = True
        if self.to_active_state is not None:
            self.to_active_state(self.shutdown_message)

    def start(self, to_active_state):
        self._to_active_state = to_active_state
        if self._is_shuting_down:
            self.to_active_state(self.shutdown_message)
```

That addresses one race condition, but there is another. What if `on_shutdown` is called while we're executing `start`? In this case, a number of problems are possible. Let's fix this by adding a mutex to our methods.

```python
from threading import Lock


class ShutdownMessageSender:
    def __init__(self):
        self._lock = Lock()
        self._to_active_state = None
        self._is_shuting_down = False
        self.shutdown_message = {"type": "shutdown", "data": True}
        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        with self._lock:
            self._is_shuting_down = True
            if self.to_active_state is not None:
                self.to_active_state(self.shutdown_message)

    def start(self, to_active_state):
        with self._lock:
            self._to_active_state = to_active_state
            if self._is_shuting_down:
                self.to_active_state(self.shutdown_message)
```

This code is almost complete. But we need one more method to support reuse. In general, message senders are reused. This is because some message senders are expensive to create. Take for example, any message senders that subscribe to a ROS topic. These message senders maintain a set of connections and we don't want to start and stop these connections every time we change states (otherwise, our new state would needlessly stall as it reconnects). To make message senders reusable, we will add a `stop` method. We will cover message sender reuse later. For now, here's what we need:

```python
from threading import Lock


class ShutdownMessageSender:
    def __init__(self):
        self._lock = Lock()
        self._to_active_state = None
        self._is_shuting_down = False
        self.shutdown_message = {"type": "shutdown", "data": True}
        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        with self._lock:
            self._is_shuting_down = True
            if self.to_active_state is not None:
                self.to_active_state(self.shutdown_message)

    def start(self, to_active_state):
        with self._lock:
            self._to_active_state = to_active_state
            if self._is_shuting_down:
                self.to_active_state(self.shutdown_message)

    def stop(self):
        with self._lock:
            self._to_active_state = None
```

And with that, we've created a `ShutdownMessageSender` that closely resembles the one you can find in `message_senders.py`.

Next let's update our state code to use this message sender.

### Using our MessageSender to introduce Handlers

In the previous section, we created a MessageSender. In this section we'll use it. Let’s start with a naive approach:

```python
class ExampleState(smach.State):
    def __init__(self):
        self.message_queue = Queue()
        # Give our state access to a ShutdownMessageSender
        self.shutdown_message_sender = ShutdownMessageSender()

    def execute(self, userdata):
        # Start sending messages to our state when we enter this state
        self.shutdown_MessageSender.start(self.message_queue.put)
        while True:
            msg = self.message_queue.get()
            if msg["type"] == "shutdown":
                # Stop sending messages before we leave this state
                self.shutdown_message_sender.stop()
                return "exit"
```

This is perhaps the most direct way to integrate the `ShutdownMessageSender`. We give our state access to a `ShutdownMessageSender` by instantiating one in the constructor. Then we tell our `ShutdownMessageSender` to start sending us messages when we enter `execute`. And right before we leave `execute`, we tell our message sender to stop sending messages.

The problem we need to address now, is figuring out how we respond to the messages we receive.

> There are other problems with this code. Most notably, this example only has one message sender. We will cover that problem in more detail later (when we fix it). For now, it's enough to see how one message sender integrates with a state.

Let’s talk about the way we respond to messages.  Within our execute method, we can check to see if our message is a "shutdown" message. If it is, we run some clean up code and transition out of the state.

But consider that every state needs the ability to exit when the process wants to shut down. And there will be many more abilities that all states will need. We don't want to repeat ourselves by putting the same snippets of code all over the place. What we need are some good abstractions. Let's find the abstractions we need by considering the code for shutdown. We need something functionally equivalent to this snippet:

```python
if message["type"] == "shutdown":
    transition("exit")
```

Let’s break down what we’re doing. This `if` statement has two parts. The conditional expression and the suite. The conditional is _selecting_ the block of code that we want to run in response to a shutdown message. But in general, we’re going to need a way of selecting the subroutines that _handle_ our responses for all messages. Besides the conditional, we also have the suite. The suite performs the particular actions needed when we receive a shutdown message. In this case, it's _handling_ the message by causing us to transition with the ‘exit’ event. But for every message, we're going to need some arbitrary code that _handles_ our response. Thus we've identified two units of abstraction:

1. An abstraction that selects the code we want to run when we receive a message.
1. An abstraction that responds to a message.

We need a specialized collection (for selecting), and handlers (for responding).

> Notice how the code that stops the message sender was removed in the previous code block. When identifying abstractions, boundaries are critical. We're trying to capture the essence of responding to messages. The details of how a state prepares for transition should be encapsulated in the state.

The specialized collection, calls all the subroutines responsible for handling a given type of message.

The handler is a method or a function that takes the message as input and does something useful. It has the ability to transition to a new state or not. In general, it can do anything (so long as it doesn't block the thread for too long).

> The word "subroutine" is used as a catch-all term for all [callable objects](https://docs.python.org/3/reference/expressions.html#calls) (like functions and methods).

Let's create an interface for our specialized collection. At its core is a collection of handlers. Therefore, we need a way to add handlers. And when we add them, we'll need to tell our object what type of messages it can handle. Finally, we need a way to notify all the relevant handlers when we receive a message. Thus we need two methods:

```python
class MessageHandlers:
    def add(message_type, handler):
        """Add a handler to the collection.

        The message_type parameter is a string. It identifies the type of
        message this handler takes.

        The handler parameter is a callable object. This will be called
        with one argument where the argument is a message.
        """
        pass

    def notify(message):
        """Calls the handlers associated with given type of message

        This will pass the given message as an argument when calling the
        handlers.

        The message parameter is a dict. It would have arrived from the state's
        message queue. It should have a key/value entry for 'type' and 'data'.
        """
        pass
```

The interface for our collection of `MessageHandlers` determines the interface for our handlers.

```python
def handler(message):
    """Handles a message.

    Returns a str to cause the state to transition. Otherwise it returns None
    """
    pass
```

Next we will integrate `MessageHandlers` with our states.

### Using MessageHandlers

A `MessageHandlers` class has been written. To use it, extend `BaseState` and call `super().__init__()` before accessing it in the field called `self.handlers`. Here's an example:

```python
import rospy
from .BaseState import BaseState


class Arm(BaseState):
    """This state arms the drone.

    The 'error' outcome will occur if drone.arm() returns false
    The 'succeeded' outcome will occur once it receives a state message with armed == True
    """

    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(**kwargs)

        self.message_senders.append(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)

    def on_entry(self, userdata):
        rospy.loginfo("arming")
        is_success = self.drone.arm()
        if is_success is not None and is_success == False:
            rospy.logfatal("could not arm")
            return "error"

    def on_state_message(self, message):
        is_message_ok = all([message["type"] == "state", message["data"] is not None])
        if not is_message_ok:
            return
        drone_state = message["data"]
        if drone_state.armed:
            return "succeeded"
```


## Some Background

The autonomous pilot software needs data from many sources including:

- The flight controller
- ROS
- The local MQTT broker
- The cloud MQTT Broker
- Sensors directly connected to the companion computer (like the camera)

These sources provide data that our state machine needs to be responsive to. And we need to accept data from all of them at the same time.

Beyond that, the autopilot software runs a state machine and many of the states need to respond to some of this data in the same way. ~~We use handlers to bundle a data sources and a common response.~~

## How do states work?
At the center of the state is its execute method. It receives messages and looks at them to decide when the state should transition.

The execute methods work by reading a queue of messages. ~~The handlers take care of getting data to that queue. When the state starts, it tells all the handlers to register their callbacks. The callbacks take a message from an external source (like the flight controller or MQTT) and. They push those messages to the state’s queue.~~

~~An example makes it easier.~~

~~Suppose we have a hello world state. It has an MQTT handler. The state starts off by tell the MQTT handler to register it’s callback. Then it waits for a message. When it gets a message,  it gives the handler a chance to respond with a general method. In the case the MQTT handler will look at the message and put it in a field (more on what the real MQTT handler does later.  it decide if we need to leave this state for a general reason. After the general case the state itself has a chance to respond.~~



## ~~What are handlers?~~



~~They connect the state to a stream of messages. They take care of handling the general way a given type of message should be handled.~~

~~For example, the position handler connects to the flight controller. It receives position messages. In general when we receive new position data we set a field so we can look at it later.~~

~~Handlers typically work by registering a callback. ROS lets us register callbacks that get called when MAVROS pushes a message. The MQTT interface lets us register a callback and it gets called whenever we receive a message from the MQTT broker.~~

~~All the states need to register their callbacks in the same way. And they tend to do the same things with the data.~~
