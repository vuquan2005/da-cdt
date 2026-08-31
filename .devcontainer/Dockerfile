FROM osrf/ros:humble-desktop-jammy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        curl \
        mesa-utils \
        libgl1-mesa-dri \
        libegl1 \
        libgl1 \
        python3-colcon-common-extensions \
        python3-vcstool \
        ros-humble-ros-gz \
        ros-humble-nav2-bringup \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        ros-humble-teleop-tools \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ign-ros2-control \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-transport-plugins \
        python3-pip \
        python3-opencv \
        joystick \
        evtest \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir "numpy<2.0.0" "setuptools<80" "packaging>=22.0" "opencv-python<5" ultralytics \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video,input,messagebus ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME}/.config/Ultralytics \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.config

RUN rosdep init || true

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /workspaces/ros-cdt
