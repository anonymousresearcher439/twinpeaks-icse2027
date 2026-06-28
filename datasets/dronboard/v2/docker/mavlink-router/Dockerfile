FROM ubuntu:22.04 AS builder

# install the build tools
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    meson \
    ninja-build \
    pkg-config \
    gcc \
    g++ \
    systemd \
    && apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# get the mavlink-router source code
RUN git clone https://github.com/mavlink-router/mavlink-router.git
WORKDIR /mavlink-router
RUN git submodule update --init --recursive

# build mavlink-router and install it
RUN meson setup build . \
    && ninja -C build \
    && ninja -C build install


# Move the executable to a smaller container that doesn't have all the build tools
FROM ubuntu:22.04

COPY --from=builder /usr/bin/mavlink-routerd /usr/bin/mavlink-routerd

# copy the config file
COPY main.conf /etc/mavlink-router/main.conf
# we define our runtime config by editing main.conf
# we can specify UDP and UART endpoints in the conf file
# we need to mount the TTY device to the container

# here's how we might start the container:
CMD /usr/bin/mavlink-routerd

# or we can use CLI args to specify the details
# CMD mavlink-router --endpoint 10.223.0.1:14550 --endpoint /dev/ttyGCS:921600