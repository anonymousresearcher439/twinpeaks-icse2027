#
# PX4 base development environment
#

FROM ubuntu:20.04 AS dev-base-focal
LABEL maintainer="Daniel Agar <daniel@agar.ca>"

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update 
RUN apt-get -y --quiet --no-install-recommends install \
    bzip2 \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    dirmngr \
    doxygen \
    file \
    g++ \
    gcc \
    gdb \
    git \
    gnupg \
    gosu \
    lcov \
    libfreetype6-dev \
    libgtest-dev \
    libpng-dev \
    libssl-dev \
    lsb-release \
    make \
    ninja-build \
    openjdk-8-jdk \
    openjdk-8-jre \
    openssh-client \
    pkg-config \
    python3-dev \
    python3-pip \
    rsync \
    shellcheck \
    sudo \
    tzdata \
    unzip \
    valgrind \
    wget \
    xsltproc \
    zip 

RUN apt-get -y autoremove 
RUN apt-get clean autoclean 
RUN rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# gtest
RUN cd /usr/src/gtest \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && find . -name \*.a -exec cp {} /usr/lib \; \
    && cd .. && rm -rf build

# Install Python 3 pip build dependencies first.
RUN python3 -m pip install --upgrade pip wheel setuptools

# Python 3 dependencies installed by pip
RUN python3 -m pip install argparse argcomplete coverage cerberus empy==3.3.4 jinja2 kconfiglib \
        matplotlib==3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
        pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl

# manual ccache setup
RUN ln -s /usr/bin/ccache /usr/lib/ccache/cc \
    && ln -s /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
    && cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
    && make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
    && rm -rf /tmp/*

# Gradle (Required to build Fast-RTPS-Gen)
RUN wget -q "https://services.gradle.org/distributions/gradle-6.3-rc-4-bin.zip" -O /tmp/gradle-6.3-rc-4-bin.zip \
    && mkdir /opt/gradle \
    && cd /tmp \
    && unzip -d /opt/gradle gradle-6.3-rc-4-bin.zip \
    && rm -rf /tmp/*

ENV PATH "/opt/gradle/gradle-6.3-rc-4/bin:$PATH"

# Intall foonathan_memory from source as it is required to Fast-RTPS >= 1.9
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory \
    && cd /tmp/foonathan_memory \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . --target install -- -j $(nproc) \
    && rm -rf /tmp/*

# Fast-DDS (Fast-RTPS 2.0.2)
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 /tmp/FastDDS-2.0.2 \
    && cd /tmp/FastDDS-2.0.2 \
    && mkdir build && cd build \
    && cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
    && cmake --build . --target install -- -j $(nproc) \
    && rm -rf /tmp/*

# Fast-RTPS-Gen 1.0.4
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /tmp/Fast-RTPS-Gen-1.0.4 \
    && cd /tmp/Fast-RTPS-Gen-1.0.4 \
    && gradle assemble \
    && gradle install \
    && rm -rf /tmp/*

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user \
    && usermod -a -G dialout,sudo user

# setup virtual X server
RUN mkdir /tmp/.X11-unix \
    && chmod 1777 /tmp/.X11-unix \
    && chown -R root:root /tmp/.X11-unix
ENV DISPLAY :99

ENV CCACHE_UMASK=000
ENV FASTRTPSGEN_DIR="/usr/local/bin/"
ENV PATH="/usr/lib/ccache:$PATH"
ENV TERM=xterm
ENV TZ=UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# create and start as LOCAL_USER_ID
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]

##########################################################################################

#
# Gazebo 11 + gzweb development environments (Ubuntu 20.04 Focal)
#

FROM dev-base-focal AS  gazebo11-gzweb-focal
LABEL maintainer="Nuno Marques <nuno.marques@dronesolutions.io>"

RUN wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-get update --fix-missing \
    && DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
        ant \
        binutils \
        bc \
        dirmngr \
        gazebo9 \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
        libeigen3-dev \
        libgazebo9-dev \
        libgstreamer-plugins-base1.0-dev \
        libimage-exiftool-perl \
        libopencv-dev \
        libxml2-utils \
        screen \
        mesa-utils \
        protobuf-compiler \
        x-window-system \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Some QT-Apps (like Gazebo) do not work without this
ENV QT_X11_NO_MITSHM 1

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
        python \
        libjansson-dev \
        libboost-dev \
        imagemagick \
        libtinyxml-dev \
        mercurial \
        cmake \
        build-essential \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# 
# Install NODE JS 11.15
WORKDIR /tmp
RUN set -eux; \
    CPU_ARCHITECTURE="$(dpkg --print-architecture)"; \
    case "${CPU_ARCHITECTURE##*-}" in \
        amd64) nodejs=node-v11.15.0-linux-x64 ;; \
        arm64) nodejs=node-v11.15.0-linux-arm64 ;; \
        *) echo >&2 "error: unsupported architecture: ${CPU_ARCHITECTURE}"; exit 1 ;; \
    esac; \
    echo $nodejs \
    && wget -O nodejs.tar.xz "https://nodejs.org/download/release/v11.15.0/$nodejs.tar.xz" \
    && tar -C /usr/local -xf "nodejs.tar.xz" \
    && rm "/tmp/nodejs.tar.xz"
# TODO only add one nodejs directory to the path
ENV PATH "/usr/local/node-v11.15.0-linux-x64/bin:/usr/local/node-v11.15.0-linux-arm64/bin:$PATH"

# Instal gzweb
WORKDIR /usr/local
RUN git clone --branch gzweb_1.4.1 https://github.com/osrf/gzweb

# install ardupilot_gazebo
WORKDIR /usr/local
RUN git clone --branch dev https://github.com/khancyr/ardupilot_gazebo
# This script will compile and install ardupilot_gazebo 
COPY --chmod=554 ./scripts/build_ardupilot_gazebo.sh /tmp/build_ardupilot_gazebo.sh
RUN /tmp/build_ardupilot_gazebo.sh /usr/local/ardupilot_gazebo

# copy some scripts (we need them later)
COPY --chmod=555 ./scripts/download_models.sh /tmp/download_models.sh
COPY --chmod=555 ./scripts/download_models.py /tmp/download_models.py



# WORKDIR /

##########################################################################################

FROM gazebo11-gzweb-focal

RUN echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user_no_passwd
RUN chmod 0440 /etc/sudoers.d/user_no_passwd
WORKDIR /home/user
USER user

# Download models for gazebo
# Note we must run this as `user` or the models will only be available to root
RUN /tmp/download_models.sh /tmp/download_models.py

# setup gzweb
COPY --chmod=555 ./scripts/setup_gzweb.sh /tmp/setup_gzweb.sh
USER root
RUN /tmp/setup_gzweb.sh /usr/local/gzweb
USER user

RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/user/ardupilot
RUN . $HOME/.profile && USER=user ./Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN ./waf configure --board sitl 
RUN ./waf copter
# do I need this:???

# this next environment variable is used by sim_vehicle.py
# normally sim_vehicle.py will try to open a GUI terminal window when it starts
# ardupilot. But in docker we don't have a GUI, so well use screen instead:
ENV SITL_RITW_TERMINAL="screen -D -m" 

RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
RUN echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

##########################################################################################
# Troubleshooting packages:
# provides:
# - `ip` as in `ip addr` or `ip link` (from iproute2)
# - ping (from iputils-ping)
# - netstat for commands like `route`,  `ifconfig`, and `netstat -i`
# - `vim` 
##########################################################################################
RUN sudo apt-get update --fix-missing \
    && DEBIAN_FRONTEND=noninteractive sudo apt-get -y --quiet --no-install-recommends install \
        iproute2 \
        iputils-ping \
        net-tools \
        vim \
    && sudo apt-get -y autoremove \
    && sudo apt-get clean autoclean \
    && sudo rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN echo 'source /usr/share/gazebo-9/setup.sh' >> ~/.profile

COPY --chmod=555 ./scripts/start_simulation.sh /usr/local/bin/start_simulation.sh