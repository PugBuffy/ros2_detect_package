FROM nvidia/cuda:12.0.0-devel-ubuntu22.04

ARG distro="humble"

WORKDIR /root

# Install necessary software for the installation of ROS2
RUN apt-get update && apt-get install -y \ 
                      locales \
                      curl \
                      gnupg2 \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Setup of the locale
RUN locale-gen ja_JP ja_JP.UTF-8 \
    && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
    && export LANG=ja_JP.UTF-8

# Add key
RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http:packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt update

# Specify the distribution of ROS2
ENV ROS_DISTRO $distro
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
RUN apt-get install -y software-properties-common

RUN add-apt-repository universe

RUN apt install -y python3-colcon-common-extensions
RUN apt install -y python3-rosdep
RUN apt install -y python3-argcomplete

RUN apt update && apt upgrade -y

RUN apt install -y ros-$ROS_DISTRO-ros-base 
# RUN apt install rm -rf /var/lib/apt/lists/*rm

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
# Set the entry point
COPY . /root

RUN apt-get update && apt-get install -y python3 python3-pip
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y

RUN apt-get update -y

RUN apt-get install ros-${ROS_DISTRO}-cv-bridge -y
RUN apt-get install ros-${ROS_DISTRO}-vision-opencv -y

RUN apt-get update -y

RUN pip install -r requirements.txt

RUN apt-get update -y
RUN apt-get upgrade -y

