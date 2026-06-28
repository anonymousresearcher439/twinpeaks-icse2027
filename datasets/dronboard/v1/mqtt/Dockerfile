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
COPY port.conf /etc/mosquitto/conf.d/port.conf


CMD [ "/usr/sbin/mosquitto", "-c", "/etc/mosquitto/mosquitto.conf" ]