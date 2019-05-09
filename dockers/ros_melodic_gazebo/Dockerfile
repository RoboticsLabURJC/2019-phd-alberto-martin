FROM osrf/ros:melodic-desktop-full-bionic

EXPOSE 11311
EXPOSE 11345
EXPOSE 8080
EXPOSE 7681


RUN curl -sL https://deb.nodesource.com/setup_8.x > setup_8.sh && chmod +x setup_8.sh && ./setup_8.sh

RUN apt update && apt upgrade -y && apt install -y \
        lsb-release \
        build-essential \
        curl \
        python-dev \
        python-pip \
        git \
        wget \
        cmake \
        nano \
        sudo \
        psmisc \
        xorg-dev \
        xvfb \
        imagemagick \
        libboost-all-dev \
        libgts-dev \
        libjansson-dev \
        libtinyxml-dev \
        mercurial \
        nodejs \
        nodejs-legacy \
        pkg-config \
    && apt clean \
    && rm -rf /var/lib/apt/list/*


RUN adduser --disabled-password --gecos '' docker &&\
    adduser docker sudo &&\
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker

RUN sudo apt update && sudo apt install -y \
        python-catkin-tools \
        ros-melodic-turtlebot3 \
        ros-melodic-turtlebot3-gazebo \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/list/* \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && . ~/.bashrc \
    && rosdep update

ENV GAZEBO_MODEL_PATH /home/docker/.gazebo/models
WORKDIR /home/docker/

RUN mkdir -p $HOME/.gazebo/models && wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz &&\
    tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1 &&\
    rm /tmp/default.tar.gz

RUN cd ~ && git clone https://github.com/RoboticsURJC-students/2019-phd-alberto-martin.git devel && \
    cd devel/gym-pyxis && cp -r gym_pyxis/envs/gazebo/assets/models ~/.gazebo && \
    cp -r /opt/ros/melodic/share/turtlebot3_description ~/.gazebo/models && \
    cp -r /opt/ros/melodic/share/turtlebot3_gazebo/models ~/.gazebo/

RUN cd ~ && git clone https://github.com/JdeRobot/assets.git jderobot_assets && \
    cd jderobot_assets && cp -r gazebo/models ~/.gazebo

ENV GZWEB_WS /home/docker/gzweb
# build gzweb
RUN cd && hg clone https://bitbucket.org/osrf/gzweb $GZWEB_WS && \
    cd $GZWEB_WS && \
    hg up default  &&\
    xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m -t

COPY ./entrypoint.sh .
ENTRYPOINT ["./entrypoint.sh"]

# xvfb-run -s '-screen 0 1280x1024x24' /bin/bash
# docker run -ti --rm -p 11311:11311 -p 11345:11345 -p 8080:8080 -p 7681:7681 ros_melodic_gazebo
# docker run -ti --rm  -net=host ros_melodic_gazebo