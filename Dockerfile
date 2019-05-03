FROM ubuntu:18.04

EXPOSE 11311
EXPOSE 11345
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
        psmisc \
        xorg-dev \
        xvfb \
    && apt clean \
    && rm -rf /var/lib/apt/list/*

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --autoremove -y curl && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130
ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION \
        cuda-compat-10-0 && \
    ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=410,driver<411"
ENV NCCL_VERSION 2.4.2
ENV CUDNN_VERSION 7.5.0.56

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-$CUDA_PKG_VERSION \
        cuda-libraries-dev-$CUDA_PKG_VERSION \
        cuda-nvtx-$CUDA_PKG_VERSION \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-minimal-build-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
        libnccl2=$NCCL_VERSION-1+cuda10.0 \
        libnccl-dev=$NCCL_VERSION-1+cuda10.0 && \
    apt-mark hold libnccl2 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
            libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

RUN adduser --disabled-password --gecos '' docker &&\
    adduser docker sudo &&\
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
WORKDIR /home/docker
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1 && \
    sudo update-alternatives --set python /usr/bin/python3.6 && \
    sudo update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1 && \
    sudo update-alternatives --set pip /usr/bin/pip3
RUN sudo -H pip3 install -U trollius rosdep rosinstall_generator rosinstall catkin_pkg catkin-tools pyside2
RUN sudo rosdep init && rosdep update
RUN sudo pip3 uninstall vcstools && sudo pip3 uninstall wstool
RUN git clone https://github.com/tkruse/vcstools &&  \
    cd vcstools && git checkout mock_server_tar_test && \
    sudo python3 setup.py develop && cd .. && \
    git clone https://github.com/vcstools/wstool && \
    cd wstool && python3 setup.py develop

RUN cd ~/ros_catkin_ws && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    cd ~/ros_catkin_ws && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cd ~/ros_catkin_ws && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

RUN cd && mkdir ~/ros_catkin_ws && cd ros_catkin_ws && \
    rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall && \
    wstool init -j8 src melodic-desktop.rosinstall

ENV ROS_PYTHON_VERSION 3
RUN sudo apt install -y libgpgme-dev python3-empy libeigen3-dev python3-sip python3-sip-dev libyaml-cpp-dev libboost-python-dev unzip python3-numpy

RUN rosdep install --from-paths src --ignore-src --rosdistro melodic -y && \
    ./src/catkin/bin/catkin_make_isolated -DPYTHON_VERSION=3.6 --install -DCMAKE_BUILD_TYPE=Release && \
    . ~/ros_catkin_ws/install_isolated/setup.bash

WORKDIR /home/docker
RUN git clone https://github.com/RoboticsURJC-students/2019-phd-alberto-martin.git devel && \
    cd devel/gym-pyxis && sudo python setup.py install && mkdir ~/.gazebo && cp -r gym_pyxis/envs/gazebo/assets/models ~/.gazebo
CMD ["bash"]

# xvfb-run -s '-screen 0 1280x1024x24' /bin/bash
#docker run -ti --rm -p 11311:11311 -p 11345:11345 pyxis