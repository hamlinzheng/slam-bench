#!/usr/bin/env bash
# Source the ROS environment, then exec whatever command the container was given.
# Per-system workspaces are sourced by scripts/run_system.sh, not here (each run
# sources exactly one system's devel/ to avoid overlay cross-talk).
set -e
source /opt/ros/noetic/setup.bash
exec "$@"
