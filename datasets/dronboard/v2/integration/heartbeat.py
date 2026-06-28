import json
import subprocess
import time
import paho.mqtt.client as mqtt

from threading import Thread


class MQTTHeartbeatSpoof(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.client = mqtt.Client(client_id="test_heartbeat")
        self._kill_thread = False

    def mqtt_connect(self, broker_address="mqtt", broker_port=1883):
        self.client.connect(broker_address, broker_port)
        self.client.loop_start()
        print("MQTTHeartbeatSpoof - connected to mqtt server")


    def mqtt_disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        print("MQTTHeartbeatSpoof - disconnected from mqtt server")


    def mqtt_start_heartbeat(self, service_id=0, seconds_between_messages=1):
        self._publish_heartbeat = True
        data = {
            "service_id" : service_id,
            "heartbeat" : time.monotonic()
        }
        print("MQTTHeartbeatSpoof - starting heartbeat")
        while True:
            while self._publish_heartbeat:
                print("MQTTHeartbeatSpoof - publishing heartbeat")
                data["heartbeat"] = time.monotonic()
                self.client.publish("heartbeat", json.dumps(data), qos=2)
                time.sleep(seconds_between_messages)
            if self._kill_thread:
                break
            

    def run(self):
        self.mqtt_start_heartbeat()


    def mqtt_stop_heartbeat(self):
        self._publish_heartbeat = False
        print("mqtt_start_heartbeat - stopped heartbeat")


    def mqtt_restart_heartbeat(self):
        self._publish_heartbeat = True
        print("mqtt_start_heartbeat - restarted heartbeat")


    def kill_thread(self):
        self._kill_thread = True


class MQTTHeartbeatDisplay(Thread):
    def __init__(self, client):
        Thread.__init__(self)
        self.client = client


    def mqtt_display_heartbeat(self):
        '''
        prints heartbeat messages to terminal
        '''
        def _print_message(client, userdata, message):
            print(f"topic: {message.topic}\npayload: {message.payload}")
        self.client.message_callback_add("heartbeat", _print_message)
        self.client.subscribe("heartbeat", qos=0)


    def run(self):
        self.mqtt_display_heartbeat()


def main():
    mqtt_broker_address = str(input("Enter the mqtt server IP address [mqtt]: "))
    if not mqtt_broker_address:
        mqtt_broker_address = "mqtt"

    try:
        heartbeat_spoof = MQTTHeartbeatSpoof()
        '''
            Need to look up mqtt container IP:
            docker inspect dr-onboardautonomy-mqtt-1 | grep IP
        '''
        heartbeat_spoof.mqtt_connect(broker_address=mqtt_broker_address)
        heartbeat_spoof.start()

        # heartbeat_display = MQTTHeartbeatDisplay(heartbeat_spoof.client)
        # heartbeat_display.start()

        subprocess.check_call("cat /catkin_ws/src/dr_onboard_autonomy/missions/test-better-circle.json | mosquitto_pub -h mqtt -t 'drone/Polkadot/mission-spec' -s", shell=True)

        '''
        Assumes hover_threshold = 20 and rtl_threhold = 60
        '''
        time.sleep(60)
        heartbeat_spoof.mqtt_stop_heartbeat()
        print("Drone should move into HeartbeatHover state")
        time.sleep(40)
        heartbeat_spoof.mqtt_restart_heartbeat()
        print("Drone should continue with state previous to HeartbeatHover")
        time.sleep(20)
        heartbeat_spoof.mqtt_stop_heartbeat()
        print("Drone should move into HeartbeatHover state again")
        time.sleep(40)
        heartbeat_spoof.mqtt_restart_heartbeat()
        print("Drone should continue with state previous to second HeartbeatHover")
        time.sleep(20)
        heartbeat_spoof.mqtt_stop_heartbeat()
        print("Drone should move into an Rtl state and no longer process heartbeat messages")
        time.sleep(70)
        heartbeat_spoof.mqtt_restart_heartbeat()
        print("Heartbeats started again, but drone should not continue as already in Rtl")
        time.sleep(30)
 
    finally:
        heartbeat_spoof.mqtt_disconnect()
        heartbeat_spoof.kill_thread()

'''
TODO - mission json and ip of mqtt server are inputs
- might be able to look up computers IP, so don't need to ask user
'''
'''
TODO - need to add in test to stop heartbeat long enough to RTL
'''
    


if __name__=="__main__":
    main()