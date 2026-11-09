# ROS 2 Humble dev environment for RGB-LiDAR-Fusion.
# Toolchain only: the repo is bind-mounted at /ws/src/rgb_lidar_fusion by docker/run.sh.
FROM osrf/ros:humble-desktop

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG LIVOX_SDK2_REF=v1.3.1
ARG LIVOX_DRIVER_REF=1.2.6

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS packages (RealSense driver, PCL, image tools) + dev / debug utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git sudo \
        python3-colcon-common-extensions python3-pip python3-rosdep \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        ros-humble-cv-bridge \
        ros-humble-image-geometry \
        ros-humble-message-filters \
        ros-humble-tf2-ros \
        ros-humble-tf2-eigen \
        ros-humble-tf2-sensor-msgs \
        libpcl-dev \
        iproute2 iputils-ping tcpdump usbutils \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2 (no apt package)
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 && git checkout "${LIVOX_SDK2_REF}" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && ldconfig && rm -rf /tmp/Livox-SDK2

# livox_ros_driver2 as an underlay workspace (equivalent of its `./build.sh humble`)
RUN mkdir -p /opt/livox_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git /opt/livox_ws/src/livox_ros_driver2 \
    && cd /opt/livox_ws/src/livox_ros_driver2 && git checkout "${LIVOX_DRIVER_REF}" \
    && cp package_ROS2.xml package.xml \
    && cp -r launch_ROS2 launch \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/livox_ws \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Non-root user matching the host UID/GID, so files written to the bind-mounted repo stay yours
RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && usermod -aG video,plugdev,dialout "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p /ws/src /ws/build /ws/install /ws/log \
    && chown -R "${USER_UID}:${USER_GID}" /ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /entrypoint.sh' >> "/home/${USERNAME}/.bashrc"

USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
