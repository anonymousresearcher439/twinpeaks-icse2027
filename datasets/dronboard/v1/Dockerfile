# GEOID DOWNLOADER
#
# Get the Earth's gravitational datafile.
# Used to convert altitude to/from height above sea-level and height above the ellipsoid)
FROM ubuntu:20.04 as geoid-downloader
RUN apt-get update && \
    apt-get install -y geographiclib-tools && \
    geographiclib-get-geoids egm96-5

# BASE
#
# This image includes all of the dependencies to build and run the project.
# by installing all the packages in a base image, we don't need to re-download
# everything to rebuild
FROM ros:noetic-robot AS base

RUN apt-key adv \
        --keyserver 'hkp://keyserver.ubuntu.com:80' \
        --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get dist-upgrade --yes \
    && apt-get install --yes \
        git \
        iproute2 \
        iputils-ping \
        mosquitto-clients \
        python-is-python3 \
        python3 \
        python3-catkin-tools \
        python3-dev \
        python3-lxml \
        python3-pip \
        python3-venv \
        python3-wheel \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-smach \
        vim \
    && rm -rf /var/lib/apt/lists/*

# install python requirements
COPY ./requirements.txt /requirements.txt
# On the Jetson (and other ARM CPUs), Ruckig must be built from source.
# In order to compile, pybind11 and pybind11-global must be installed before running
# pip install ruckig (note: ruckig is installed by the requirements.txt file)
RUN pip install "pybind11==2.9.2" "pybind11-global==2.9.2"
RUN pip install -r /requirements.txt

#
# BUILDER
# creates the ROS install
FROM base as builder

#create catkin workspace
COPY . /catkin_ws/src/dr_onboard_autonomy
WORKDIR /catkin_ws
# RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/tmp/ros/noetic
RUN /ros_entrypoint.sh catkin_make

#?? RUN . devel/setup.sh
#source noetic setup
#TODO - fix sourcing error
# RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc

#install geographiclib
COPY --from=geoid-downloader /usr/share/GeographicLib/geoids /usr/share/GeographicLib/geoids

# ENV PYTHONPATH="$PYTHONPATH:/catkin_ws/devel/lib/python3/dist-packages"
# ENV PYTHONPATH="$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages"
# ENV PYTHONPATH="$PYTHONPATH:/catkin_ws/src/dr_onboard_autonomy/src"
# ENV PYTHONPATH="$PYTHONPATH:/catkin_ws/src/dr_onboard_autonomy/test"
# ENV PYTHONPATH="$PYTHONPATH:/catkin_ws/src/dr_onboard_autonomy"
ENV ROS_VERSION=1
ENV PKG_CONFIG_PATH=/catkin_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig
ENV ROS_PYTHON_VERSION=3
ENV ROS_PACKAGE_PATH=/catkin_ws/src:/opt/ros/noetic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=/catkin_ws/devel/share/common-lisp
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV CMAKE_PREFIX_PATH=/catkin_ws/devel:/opt/ros/noetic
ENV PYTHONPATH=/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/catkin_ws/src/dr_onboard_autonomy/src:/catkin_ws/src/dr_onboard_autonomy/test:/catkin_ws/src/dr_onboard_autonomy
ENV LD_LIBRARY_PATH=/catkin_ws/devel/lib:/opt/ros/noetic/lib
ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_DISTRO=noetic


#
# FINAL STAGE
# FROM base

# COPY --from=geoid-downloader /usr/share/GeographicLib/geoids /usr/share/GeographicLib/geoids
# COPY --from=builder /tmp/ros/noetic /opt/ros/noetic


# WORKDIR /catkin_ws

# WORKDIR /tmp
# RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
# RUN sudo bash install_geographiclib_datasets.sh

#add DR-OnboardAutonomy to catkin src
#alternatively and mount to host while running container with --mount flag
# COPY / /catkin_ws/src/dr_onboard_autonomy

#Running px4 on host, so leaving commented out
# WORKDIR /catkin_ws/src/px4/
# RUN /usr/bin/bash Tools/setup/ubuntu.sh
# RUN /usr/bin/make px4_sitl_default

#source workspace setup on every bash load
# RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc
#TODO - add / modify .dockerignore

#docker build -t onboard
#docker create --name onboard -it onboard
#docker start onboard
#docker exec -it onboard /bin/bash
#roslaunch dr_onboard_autonomy onboard_pilot_mavros_px4_simulator.launch uav_name:="green" latitude:="0.2323" longitude:="39.0" altitude:=200 mqtt_host:="192.168.0.15"

