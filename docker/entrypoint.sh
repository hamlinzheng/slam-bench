#!/usr/bin/env bash
# Source the ROS environment, then exec whatever command the container was given.
# Per-system workspaces are sourced by scripts/run_system.sh, not here (each run
# sources exactly one system's devel/ to avoid overlay cross-talk).
set -e
source /opt/ros/noetic/setup.bash

# The container runs as root while /slam-bench and its submodules are owned by the host
# user, so git refuses them as "dubious ownership". Without this, every git-derived
# provenance field in metrics.json (system_commit, *_dirty) would silently be null.
git config --global --add safe.directory '*' 2>/dev/null || true

exec "$@"
