# slam-bench unified base image: ROS Noetic + PCL/Eigen (desktop-full) + the shared
# toolchain for the LIO baselines (glog, gflags, yaml-cpp, TBB) and evo for trajectory
# metrics. Systems are built into per-system catkin workspaces at run time by
# scripts/build_systems.sh (mounted, not baked), so source edits don't rebuild the image.
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget ninja-build \
      python3-pip python3-catkin-tools \
      libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libtbb-dev \
      ros-noetic-eigen-conversions ros-noetic-pcl-ros \
  && rm -rf /var/lib/apt/lists/*

# Ceres 2.2.0 from source: BIEVR-LIO needs ceres::Manifold, which focal's libceres-dev
# (1.14) predates. Flags mirror the upstream docker/scripts/install_ceres.sh.
RUN git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git /opt/ceres \
  && cmake -S /opt/ceres -B /opt/ceres/build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
       -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF \
       -DBUILD_SHARED_LIBS=ON -DSUITESPARSE=ON -DEIGENSPARSE=ON -DEIGENMETIS=ON \
       -DUSE_CUDA=OFF -DLAPACK=ON -DSCHUR_SPECIALIZATIONS=ON -DCUSTOM_BLAS=ON \
  && cmake --build /opt/ceres/build --parallel "$(nproc)" \
  && cmake --install /opt/ceres/build && ldconfig && rm -rf /opt/ceres

# evo: TUM-trajectory metrics / plots (artifact ①).
RUN pip3 install --no-cache-dir evo

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
