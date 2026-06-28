#!/usr/bin/env python3
import json
import subprocess
import time
from random import randint
import paho.mqtt.client as mqtt

from threading import Thread


class MQTTVisionSpoof(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.client = mqtt.Client(client_id=f"test_vision_{randint(0, 1000)}")
        self._kill_thread = False

    def mqtt_connect(self, broker_address="mqtt_local", broker_port=1883):
        self.client.connect(broker_address, broker_port)
        self.client.loop_start()
        print("MQTTVisionSpoof - connected to mqtt server")


    def mqtt_disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        print("MQTTVisionSpoof - disconnected from mqtt server")


    def mqtt_start_vision(self, seconds_between_messages=1):
        self._publish_vision = True
        data = {
            "x": 304,
            "y": 304,
            "x_res": 608,
            "y_res": 608,
        }
        print("MQTTVisionSpoof - starting vision messages")
        while True:
            while self._publish_vision:
                print("MQTTVisionSpoof - publishing vision message")
                input("press enter to send")
                data['ts'] = time.time()
                self.client.publish("vision", json.dumps(data), qos=2)
            if self._kill_thread:
                break
            

    def run(self):
        self.mqtt_start_vision()


    def mqtt_stop_vision(self):
        self._publish_vision = False
        print("mqtt_start_vision - stopped vision")


    def mqtt_restart_vision(self):
        self._publish_vision = True
        print("mqtt_start_vision - restarted vision")


    def kill_thread(self):
        self._kill_thread = True


class MQTTVisionDisplay(Thread):
    def __init__(self, client: mqtt.Client):
        Thread.__init__(self)
        self.client = client


    def mqtt_display_vision(self):
        '''
        prints vision messages to terminal
        '''
        def _print_message(client, userdata, message):
            print(f"topic: {message.topic}\npayload: {message.payload}")
        self.client.message_callback_add("vision", _print_message)
        self.client.subscribe("vision", qos=0)


    def mqtt_display_vision_cleanup(self):
        self.client.unsubscribe("vision")


    def run(self):
        self.mqtt_display_vision()


def main():
    view_or_start = ""
    while view_or_start not in ("view", "start"):
        view_or_start = input("'view' or 'start' vision mqtt messages: ")
        if view_or_start not in ("view", "start"):
            print("\nonly 'view' and 'start' are valid inputs\n")
    try:
        vision_spoof = MQTTVisionSpoof()
        vision_spoof.mqtt_connect()
        if view_or_start == "start":
            vision_spoof.start()
        if view_or_start == "view":
            vision_display = MQTTVisionDisplay(client=vision_spoof.client)
            vision_display.start()

        # subprocess.check_call("cat /catkin_ws/src/dr_onboard_autonomy/missions/peppermint-rd-circle-target-test.json | mosquitto_pub -h mqtt -t 'drone/Polkadot/mission-spec' -s", shell=True)

        while True:
            pass

    finally:
        vision_spoof.mqtt_disconnect()
        vision_spoof.mqtt_stop_vision()
        vision_spoof.kill_thread()
        if view_or_start == "view":
            vision_display.mqtt_display_vision_cleanup()


if __name__=="__main__":
    main()
