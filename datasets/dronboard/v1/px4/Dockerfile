#
# PX4 base development environment
#

FROM ubuntu:20.04 AS px4-dev-base-focal
LABEL maintainer="Daniel Agar <daniel@agar.ca>"

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
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
		tzdata \
		unzip \
		valgrind \
		wget \
		xsltproc \
		zip \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# gtest
RUN cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

# Install Python 3 pip build dependencies first.
RUN python3 -m pip install --upgrade pip wheel setuptools

# Python 3 dependencies installed by pip
RUN python3 -m pip install argparse argcomplete coverage cerberus empy jinja2 kconfiglib \
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
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# setup virtual X server
RUN mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix
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
# PX4 Gazebo 11 development environment in Ubuntu 20.04 Focal
#

FROM px4-dev-base-focal AS px4-dev-simulation-focal
LABEL maintainer="Nuno Marques <nuno.marques@dronesolutions.io>"

RUN wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
	&& sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
	&& apt-get update --fix-missing \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		ant \
		binutils \
		bc \
		dirmngr \
		gazebo11 \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		libeigen3-dev \
		libgazebo11-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		mesa-utils \
		protobuf-compiler \
		x-window-system \
		ignition-edifice \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Some QT-Apps/Gazebo don't not show controls without this
ENV QT_X11_NO_MITSHM 1

# Use UTF8 encoding in java tools (needed to compile jMAVSim)
ENV JAVA_TOOL_OPTIONS=-Dfile.encoding=UTF8

# Install JSBSim
# RUN wget https://github.com/JSBSim-Team/jsbsim/releases/download/v1.1.1a/JSBSim-devel_1.1.1-134.focal.amd64.deb
# RUN dpkg -i JSBSim-devel_1.1.1-134.focal.amd64.deb

##########################################################################################

#
# PX4 Gazebo 11 plus gzweb development environment in Ubuntu 20.04 Focal
#
FROM px4-dev-simulation-focal AS px4-dev-simulation-gzweb
LABEL maintainer="Michael Murphy"

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
# Install NODE JS 11
WORKDIR /tmp
# This is the name of the node js archive (without hte extention) and the name
# of the "nodejs home" directory
# For X86:
# ARG nodejs=node-v11.15.0-linux-x64
# For ARM:
# ARG 

RUN set -eux; \
	\
	dpkgArch="$(dpkg --print-architecture)"; \
	case "${dpkgArch##*-}" in \
		amd64) nodejs=node-v11.15.0-linux-x64 ;; \
		arm64) nodejs=node-v11.15.0-linux-arm64 ;; \
		*) echo >&2 "error: unsupported architecture: ${dpkgArch}"; exit 1 ;; \
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

WORKDIR /tmp
COPY scripts/setup_gzweb.sh .
RUN ./setup_gzweb.sh

COPY scripts/start_simulation.sh /bin/start_simulation.sh

WORKDIR /

##########################################################################################

FROM px4-dev-simulation-gzweb

WORKDIR /home/user
USER user
RUN git clone --branch v1.13.0 --single-branch --recursive https://github.com/PX4/PX4-Autopilot.git Firmware

# Copy the PX4 rc scripts
WORKDIR /home/user/Firmware/ROMFS/px4fmu_common/init.d-posix
COPY ./px4-rc.mavlink .
COPY ./px4-rc.params .

# Change the color of the drone so it's easier to see
COPY ./bright_colored_drone.patch /home/user/Firmware/Tools/sitl_gazebo/bright_colored_drone.patch
WORKDIR /home/user/Firmware/Tools/sitl_gazebo/
RUN git apply bright_colored_drone.patch

WORKDIR /home/user/Firmware
# Build PX4
RUN bash -c "DONT_RUN=1 make px4_sitl gazebo_typhoon_h480"


USER root
RUN apt-get update && apt-get install --yes iproute2 dnsutils iputils-ping
USER user
WORKDIR /