# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install MoveIt 2, UR Driver, and common development tools
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-ur \
    ros-humble-rmw-cyclonedds-cpp \
    git \
    python3-colcon-common-extensions \
    nano \
    sudo \
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
