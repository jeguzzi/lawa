FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros-msgs \
    python-pip \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install blinkstick

COPY . src/lawa
RUN catkin build
