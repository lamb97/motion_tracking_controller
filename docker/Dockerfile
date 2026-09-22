FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates \
    python3-colcon-common-extensions python3-rosdep \
    mesa-utils xauth xvfb \
    && rm -rf /var/lib/apt/lists/*

# Upstream's Noble/Jazzy binary repositories, confined to this container.
RUN for farm in legged simulation unitree; do \
      echo "deb [trusted=yes] https://github.com/qiayuanl/${farm}_buildfarm/raw/noble-jazzy-amd64/ ./" \
        > "/etc/apt/sources.list.d/qiayuanl_${farm}.list"; \
      echo "yaml https://github.com/qiayuanl/${farm}_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" \
        > "/etc/ros/rosdep/sources.list.d/1-qiayuanl_${farm}.list"; \
    done \
    && printf 'Package: ros-jazzy-mujoco-ros2-control\nPin: origin "github.com"\nPin-Priority: 1001\n' \
      > /etc/apt/preferences.d/beyondmimic-mujoco \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-legged-control-base ros-jazzy-legged-rl-controllers ros-jazzy-legged-bringup \
      ros-jazzy-mujoco-ros2-control ros-jazzy-mujoco-sim-ros2 \
      ros-jazzy-unitree-description ros-jazzy-unitree-systems \
      ros-jazzy-rosbag2-storage-mcap ros-jazzy-joy-linux ros-jazzy-joy-teleop \
    && rm -rf /var/lib/apt/lists/*

ARG UNITREE_BRINGUP_REF=8a0f30875808c7fe940301732467de57de75babd
WORKDIR /ws
RUN git clone https://github.com/qiayuanl/unitree_bringup.git src/unitree_bringup \
    && git -C src/unitree_bringup checkout "${UNITREE_BRINGUP_REF}"

COPY . /ws/src/motion_tracking_controller
RUN source /opt/ros/jazzy/setup.bash \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && colcon build --symlink-install --parallel-workers 4 \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      --packages-up-to unitree_bringup motion_tracking_controller

COPY docker/entrypoint.sh /beyondmimic-entrypoint.sh
RUN chmod +x /beyondmimic-entrypoint.sh
ENV ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENTRYPOINT ["/beyondmimic-entrypoint.sh"]
CMD ["bash"]
