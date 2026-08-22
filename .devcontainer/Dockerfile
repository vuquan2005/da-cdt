# ROS 2 Humble targets Ubuntu 22.04 (Jammy). Gazebo Fortress is installed via
# ros_gz below; Gazebo Classic is deliberately not part of this environment.
FROM osrf/ros:humble-desktop-jammy

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        mesa-utils \
        libgl1-mesa-dri \
        libegl1 \
        libgl1 \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-ros-gz \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-xacro \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-teleop-twist-keyboard \
        ros-humble-teleop-tools \
        ros-humble-joy-teleop \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ign-ros2-control \
        joystick \
        evtest \
        sudo \
        vim \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video,input,messagebus ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep is initialized system-wide; dependency downloads are still performed
# explicitly by the developer after opening the workspace.
RUN rosdep init || true

USER ${USERNAME}
WORKDIR /workspaces/ros-cdt

# Source ROS and colcon autocomplete automatically in every interactive shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /home/${USERNAME}/.bashrc
