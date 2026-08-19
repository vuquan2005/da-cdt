# ROS 2 Humble targets Ubuntu 22.04 (Jammy). Gazebo Fortress is installed via
# ros_gz below; Gazebo Classic is deliberately not part of this environment.
FROM osrf/ros:humble-desktop-jammy

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-ros-gz \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-xacro \
        sudo \
        vim \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep is initialized system-wide; dependency downloads are still performed
# explicitly by the developer after opening the workspace.
RUN rosdep init || true

ENV DEBIAN_FRONTEND=

USER ${USERNAME}
WORKDIR /workspaces/ros-cdt

# Source ROS automatically in every interactive shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc
