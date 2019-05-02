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

WORKDIR /home/docker
RUN git clone https://github.com/RoboticsURJC-students/2019-phd-alberto-martin.git devel
RUN sudo chown docker:docker -R devel
RUN cd devel/gym-pyxis && sudo python2 setup.py install && mkdir ~/.gazebo && cp -r devel/gym_pyxis/envs/gazebo/assets/models ~/.gazebo
CMD ["bash"]

# xvfb-run -s '-screen 0 1280x1024x24' /bin/bash
#docker run -ti --rm -p 11311:11311 -p 11345:11345 pyxis