FROM ubuntu:18.04

EXPOSE 11311
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Madrid
RUN apt update && apt install -y tzdata &&\
        ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone &&\
        dpkg-reconfigure --frontend noninteractive tzdata \
    && apt clean \
    && rm -rf /var/lib/apt/list/*

RUN apt update && apt upgrade -y && apt install -y \
        lsb-release \
        build-essential \
        curl \
        python-dev \
        python3-dev \
        python-pip \
        python3-pip \
        git \
        wget \
        cmake \
        nano \
        sudo \
    && apt clean \
    && rm -rf /var/lib/apt/list/*


RUN adduser --disabled-password --gecos '' docker &&\
    adduser docker sudo &&\
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    sudo apt update && \
    sudo apt install -y ros-melodic-desktop-full && \
    sudo apt install -y python-catkin-tools \
        ros-melodic-turtlebot3 \
        ros-melodic-turtlebot3-gazebo \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/list/* \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && . ~/.bashrc \
    && sudo rosdep init && rosdep update
