"""The BaseState module. All states extend the BaseState class in this module
"""
from queue import Queue

import smach

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import RcFailsafe

from . import MessageHandler


class BaseState(smach.State):
    """All DR states extend this class. It has the execute method.
    """
    def __init__(
        self, drone=None, reusable_message_senders=None, mqtt_client=None, **kwargs
    ):
        outcomes = kwargs["outcomes"]
        # make sure every state can have the error outcome so that shutdown works
        if "error" not in outcomes:
            outcomes.append("error")
        super().__init__(outcomes=outcomes)
        self.args = kwargs
        if 'uav_id' in kwargs:
            self.uav_id = kwargs['uav_id']
        self.drone = drone
        self.reusable_message_senders = reusable_message_senders
        self.mqtt_client = mqtt_client
        self.message_data = {}
        self.message_queue = Queue()
        self.message_senders = set()
        self.handlers = MessageHandler()

        # To send the drone's status over MQTT  we need to make status messages and to make that we
        # need these message senders:
        self.data_senders = {"position", "relative_altitude", "state", "battery"}
        for message_type in self.data_senders:
            message_sender = self.reusable_message_senders.find(message_type)
            self.message_senders.add(message_sender)
            self.handlers.add_handler(message_type, self.on_data_update_message)

        self.send_data_timer = RepeatTimer("send_data_timer", 1)
        self.message_senders.add(self.send_data_timer)
        self.handlers.add_handler("send_data_timer", self.on_send_data)

        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        if "human_control" in outcomes:
            self._rc_failsafe_component = RcFailsafe(self)

        if "abort" in outcomes:
            # How to respond to a message sender
            self.message_senders.add(self.reusable_message_senders.find("abort"))
            self.handlers.add_handler("abort", self._on_abort)

    def _on_abort(self, message):
        return "abort"

    def on_entry(self, userdata):
        """The first thing that runs when we enter this state.

        This is the method calls the execute() method on other states, if the
        state implemented by running other states.
        """
        return None

    def execute(self, userdata):
        """The execute method receives messages and processes them"""
        outcome = self.on_entry(userdata)
        if outcome is not None:
            return outcome
        self.start_message_senders()

        while True:
            msg = self.message_queue.get()
            self.message_data[msg["type"]] = msg["data"]
            outcome = self.handlers.notify(msg)
            if outcome is not None:
                self.acknowledge_message(msg)
                self.stop_message_senders()
                return outcome

            outcome = self.execute2(userdata)
            self.acknowledge_message(msg)
            if outcome is not None:
                self.stop_message_senders()
                return outcome

    def execute2(self, userdata):
        """Method that runs every time we receive a message that hasn't caused
        a state transition
        """
        return None

    def start_message_senders(self):
        """Starts the message senders

        This tells the message senders to start passing their data to this
        state.
        """
        for sender in self.message_senders:
            sender.start(self.message_queue.put)

    def stop_message_senders(self):
        """Stops all the message senders.
        
        This gets called before exiting this state.
        """
        for sender in self.message_senders:
            sender.stop()

    def acknowledge_message(self, msg):
        """Calls the done_func if the message came from a reliable message
        sender.
        """
        if "done" in msg:
            done_func = msg["done"]
            done_func()

    def on_shutdown(self, message):
        return "error"

    def on_data_update_message(self, _):
        """Update the status update data

        The drone reports it's status to the ground. This method updates the
        internal data that will eventually get sent.
        """
        is_data_available = all(
            [
                "battery" in self.message_data,
                "state" in self.message_data,
                "position" in self.message_data,
                "relative_altitude" in self.message_data,
            ]
        )

        if not is_data_available:
            return

        data = self.message_data
        position = {
            "latitude": data["position"].latitude,
            "longitude": data["position"].longitude,
            "altitude": data["relative_altitude"].data,
        }
        battery = {
            "voltage": data["battery"].voltage,
            "current": data["battery"].current,
            "level": data["battery"].percentage,
        }

        self.drone.update_data("location", position)
        self.drone.update_data("battery", battery)
        self.drone.update_data("status", data["state"].system_status)
        self.drone.update_data("mode", data["state"].mode)
        self.drone.update_data("state_name", type(self).__name__)
        self.drone.update_data("armed_state", data["state"].armed)

    def on_send_data(self, _):
        """Sends status update message to the ground
        """
        data = self.drone.data.to_dict()
        self.mqtt_client.publish("update_drone", data)
