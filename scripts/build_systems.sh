#!/usr/bin/env bash
# Build each baseline in its OWN isolated catkin workspace under /ws.
#
# Isolation is deliberate (plan §11): FAST-LIO needs the workspace-level
# livox_ros_driver, while faster-lio bundles its own copy via add_subdirectory —
# two packages of the same name in one workspace would collide. One workspace per
# system sidesteps that entirely and keeps a broken build in one system from
# poisoning the others.
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

# FAST-LIO: package + the message-generating livox driver (needs Livox-SDK, in the image).
build_ws fast_lio   "$SYS/FAST_LIO" "$SYS/livox_ros_driver/livox_ros_driver"

# faster-lio: self-contained (bundles livox_ros_driver messages in thirdparty/).
build_ws faster_lio "$SYS/faster-lio"

echo "==================== all systems built ===================="
