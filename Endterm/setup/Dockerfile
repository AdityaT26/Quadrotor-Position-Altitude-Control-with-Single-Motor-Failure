# Use the base image for ROS Humble Desktop
FROM osrf/ros:humble-desktop

# Set build arguments
ARG USER=ideaForge
ARG DEBIAN_FRONTEND=noninteractive

RUN groupadd -r ${USER} && useradd -r -g ${USER} ${USER} && \
    mkdir -p /home/${USER} && \
    chown -R ${USER}:${USER} /home/${USER} && \
    echo ${USER}' ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    chmod 0440 /etc/sudoers

# Set the working directory
WORKDIR /home/${USER}

# Update package lists and install required ROS packages and tools
RUN apt-get update && \
    apt-get install -y \
    python3 \
    python3-pip \
    nano \
    git \
    wget \ 
    curl \ 
    ros-humble-gazebo-* \ 
    ros-humble-gazebo-ros-pkgs \ 
    libeigen3-dev \ 
    vim && \
    rm -rf /var/lib/apt/lists/*
# Install Gazebo & Meshlab
RUN curl -sSL http://get.gazebosim.org | sh
# PX4 Install Script
RUN wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/ubuntu.sh && \
    wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/requirements.txt && \
    chmod +x ubuntu.sh && \
    ./ubuntu.sh && \
    rm ubuntu.sh && \
    rm requirements.txt
# Install MAVROS
RUN apt-get install -y ros-humble-mavros \
    ros-humble-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install Python packages
RUN pip install --upgrade pymavlink MAVProxy

# Edit .bashrc
RUN echo '#! /bin/bash' >> /home/${USER}/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/${USER}/.bashrc

# Switch to the specified user
USER ${USER}