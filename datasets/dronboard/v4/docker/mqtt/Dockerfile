FROM ubuntu:22.04

RUN apt-get update \
    && apt-get install --yes \
        iproute2 \
        mosquitto \
        mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# /run/mosquitto/
# /var/lib/mosquitto/
# /var/log/mosquitto/
RUN mkdir -p \
        /run/mosquitto/ \
        /var/lib/mosquitto \
        /var/log/mosquitto \
    && chown mosquitto:mosquitto \
        /run/mosquitto/ \
        /var/lib/mosquitto \
        /var/log/mosquitto

EXPOSE 1883/tcp
COPY drone-response.conf /etc/mosquitto/conf.d/drone-response.conf

# We need `stdbuf -oL` to help with observation and troubleshooting.
# The `stdbuf` command is used here to modify the buffering of the Mosquitto MQTT broker's output. 
# By specifying `-oL`, the output is set to line buffering. This change ensures that Mosquitto's logs
# are flushed to stdout after every line. We need this modification because we want to know
# immediately when clients (like drones) connect or disconnect. This real-time feedback is helpful
# when developing our software, but it's also helpful in the field. 
CMD [ "stdbuf", "-oL", "/usr/sbin/mosquitto", "-c", "/etc/mosquitto/mosquitto.conf" ]
