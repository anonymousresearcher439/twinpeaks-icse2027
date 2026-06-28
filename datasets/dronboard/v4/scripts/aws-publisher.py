import time
import paho.mqtt.client as paho
import ssl 
# AWS IoT connection details
ENDPOINT = "a3dpdfmwa109lg-ats.iot.us-east-2.amazonaws.com"
CLIENT_ID = "Crimson"
TOPIC = "status_message"
CA_PATH = "test/config/mqtt/CAs.crt"
CERT_PATH = "test/config/mqtt/public.crt"
KEY_PATH = "test/config/mqtt/private.key"

# Callback when the AWS IoT Core connection is established
def on_aws_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[AWS IoT] Connected successfully!")
        # Optionally, subscribe to topics on AWS IoT Core if needed:
        client.subscribe(TOPIC)
    else:
        print(f"[AWS IoT] Connection failed with return code {rc}")

# Callback when a message is received from AWS IoT Core (if you subscribe)
def on_aws_message(client, userdata, msg):
    print(f"[AWS IoT] Received message -> Topic: {msg.topic}, Payload: {msg.payload.decode()}")

# ---------------------------
# Set up AWS IoT Core client using paho-mqtt
# ---------------------------
aws_client = paho.Client(client_id=CLIENT_ID)
aws_client.tls_set(
    ca_certs=CA_PATH,
    certfile=CERT_PATH,
    keyfile=KEY_PATH,
    cert_reqs=ssl.CERT_REQUIRED,
    tls_version=ssl.PROTOCOL_TLSv1_2,
    ciphers=None
)

aws_client.on_connect = on_aws_connect
aws_client.on_message = on_aws_message

print("Connecting to AWS IoT Core...")
aws_client.connect(ENDPOINT, port=8883, keepalive=120)
aws_client.loop_start()  # Start the network loop for AWS IoT Core
# aws_client.subscribe("status_message")

time.sleep(5)
aws_client.publish("status_message", "TESTING AWS IoT Core", qos=0)

# ---------------------------
# Set up local MQTT client
# ---------------------------
'''
local_client = paho.Client()
local_client.on_message = on_local_message

print("Connecting to local MQTT broker...")
local_client.connect("localhost", 1883, 60)
local_client.loop_start()

# Subscribe to the 'update_drone' topic on the local MQTT broker
local_client.subscribe("update_drone")
print("Subscribed to local 'update_drone' topic")'
'''

# ---------------------------
# Run indefinitely
# ---------------------------
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nDisconnecting...")
finally:
    # Disconnect AWS IoT Core client
    aws_client.loop_stop()
    aws_client.disconnect()
    print("Disconnected from AWS IoT Core")
    
    '''
    # Disconnect local MQTT client
    local_client.loop_stop()
    local_client.disconnect()
    print("Disconnected from local MQTT broker")'
    '''