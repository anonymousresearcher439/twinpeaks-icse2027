FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /tmp
RUN apt-key adv \
        --keyserver 'hkp://keyserver.ubuntu.com:80' \
        --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install --yes \
        geographiclib-tools \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-mavros-msgs \
        vim \
        wget \
        screen \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh

CMD []

#install pymavlink
# RUN pip3 install cython==0.29.24
# RUN pip3 install PyGeodesy

# COPY ./catkin_ws /catkin_ws
# WORKDIR /catkin_ws
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'
# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

