# slam-bench unified base image: ROS Noetic + PCL/Eigen (desktop-full) + the shared
# toolchain for the LIO baselines (glog, gflags, yaml-cpp, TBB) and evo for trajectory
# metrics. Systems are built into per-system catkin workspaces at run time by
# scripts/build_systems.sh (mounted, not baked), so source edits don't rebuild the image.
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget \
      python3-pip python3-catkin-tools \
      libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libtbb-dev \
      ros-noetic-eigen-conversions ros-noetic-pcl-ros \
  && rm -rf /var/lib/apt/lists/*

# evo: TUM-trajectory metrics / plots (artifact ①).
RUN pip3 install --no-cache-dir evo

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
