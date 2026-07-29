#!/usr/bin/env bash
# Source the ROS environment, then exec whatever command the container was given.
# Per-system workspaces are sourced by scripts/run_system.sh, not here (each run
# sources exactly one system's devel/ to avoid overlay cross-talk).
set -e

# Containers run as the host user (compose `user:`) so that everything they write into
# the mounted results/ is owned by that user rather than root. Such a uid has no
# /etc/passwd entry, so Docker hands it HOME=/ — and ROS then fails outright trying to
# create its dot-dir there, as does matplotlib's cache. One writable fallback covers
# both, and works for any uid without hardcoding one in the image.
[ -w "$HOME" ] || export HOME=/tmp

source /opt/ros/noetic/setup.bash

# The container runs as root while /slam-bench and its submodules are owned by the host
# user, so git refuses them as "dubious ownership". Without this, every git-derived
# provenance field in metrics.json (system_commit, *_dirty) would silently be null.
git config --global --add safe.directory '*' 2>/dev/null || true

exec "$@"
