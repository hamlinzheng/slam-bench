#!/usr/bin/env bash
# Build each baseline in its OWN isolated catkin workspace under /ws.
#
# Isolation is deliberate: FAST-LIO needs a workspace-level livox_ros_driver, while
# faster-lio bundles its own copy via add_subdirectory — two packages of the same name in
# one workspace would collide. One workspace per system sidesteps that entirely and keeps a
# broken build in one system from poisoning the others.
set -euo pipefail
source /opt/ros/noetic/setup.bash
SYS=/slam-bench/systems

build_ws () {
  local name=$1; shift
  local ws=/ws/$name
  echo "==================== building ${name} ===================="
  mkdir -p "$ws/src"
  # Refresh package symlinks (idempotent across rebuilds).
  find "$ws/src" -maxdepth 1 -type l -delete 2>/dev/null || true
  for pkg in "$@"; do
    ln -sfn "$pkg" "$ws/src/$(basename "$pkg")"
  done
  ( cd "$ws" && catkin_make -DCMAKE_BUILD_TYPE=Release )
}

# Same, but with catkin_tools. Needed when a package declares <build_type>cmake</build_type>,
# which catkin_make cannot build (BIEVR-LIO's core and ros_common do). Mixing the two tools
# is only a problem inside one workspace, and these are separate.
build_ws_catkin () {
  local name=$1; shift
  local ws=/ws/$name
  echo "==================== building ${name} ===================="
  mkdir -p "$ws/src"
  find "$ws/src" -maxdepth 1 -type l -delete 2>/dev/null || true
  for pkg in "$@"; do
    ln -sfn "$pkg" "$ws/src/$(basename "$pkg")"
  done
  ( cd "$ws" && catkin init >/dev/null \
    && catkin config --extend /opt/ros/noetic --merge-devel \
         --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null \
    && catkin build )
}

# FAST-LIO: package + Livox CustomMsg types (livox_msgs is messages only, no driver).
build_ws fast_lio   "$SYS/FAST_LIO" "$SYS/livox_msgs/livox_ros_driver"

# faster-lio: self-contained (bundles livox_ros_driver messages in thirdparty/).
build_ws faster_lio "$SYS/faster-lio"

# Point-LIO: same shape as FAST-LIO.
build_ws point_lio  "$SYS/Point-LIO" "$SYS/livox_msgs/livox_ros_driver"

# Super-LIO: the upstream repo IS a catkin workspace (src/basic + src/super_lio), so both
# packages get linked in. Self-contained on CustomMsg — it bundles the generated
# livox_ros_driver headers in 3rdparty/, so linking the workspace-level livox_ros_driver
# here would put two definitions of the same message in one workspace.
build_ws super_lio  "$SYS/Super-LIO/src/basic" "$SYS/Super-LIO/src/super_lio"

# BIEVR-LIO: core + shared ROS glue + the ROS1 interface (never interfaces/ros2 — it is an
# ament package). Plus the gen2 CustomMsg types: it dispatches on the datatype name, so the
# gen1 package alone would leave our bags unrecognised.
build_ws_catkin bievr_lio \
  "$SYS/BIEVR-LIO/BIEVR" "$SYS/BIEVR-LIO/interfaces/ros_common" "$SYS/BIEVR-LIO/interfaces/ros1" \
  "$SYS/livox_msgs/livox_ros_driver2"

echo "==================== all systems built ===================="
