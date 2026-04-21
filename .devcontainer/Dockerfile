# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's/archive.ubuntu.com/de.archive.ubuntu.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/de.archive.ubuntu.com/g' /etc/apt/sources.list

RUN sudo rm -rf /var/lib/apt/lists/*
RUN sudo apt-get clean

#Install MoveIt 2, UR Driver, and common development tools
RUN sudo apt-get update -o Acquire::ForceIPv4=true
RUN (apt-get install -y -o Acquire::ForceIPv4=true --fix-missing \
    ros-humble-moveit \
    ros-humble-ur \
    ros-humble-rmw-cyclonedds-cpp) || \
    (apt-get --fix-broken install -y -o Acquire::ForceIPv4=true && \
     apt-get install -y -o Acquire::ForceIPv4=true --fix-missing \
    ros-humble-moveit ros-humble-ur ros-humble-rmw-cyclonedds-cpp)


RUN apt-get update -o Acquire::ForceIPv4=true -o Acquire::Retries=5 && \
    (apt-get install -y -o Acquire::ForceIPv4=true --fix-missing \
    git \
    python3-colcon-common-extensions \
    nano \
    sudo \
    docker.io || \
    (apt-get --fix-broken install -y -o Acquire::ForceIPv4=true && \
     apt-get install -y -o Acquire::ForceIPv4=true --fix-missing \
    git python3-colcon-common-extensions nano sudo docker.io)) \
    && rm -rf /var/lib/apt/lists/*
    
# Add a default non-root user for VS Code Dev Containers
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Automatically source ROS 2 setup in bash
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspace
