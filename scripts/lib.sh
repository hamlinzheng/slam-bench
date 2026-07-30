# Shared between the host entry point (./run.sh) and the container-side runner
# (scripts/run_system.sh), so the preset resolution rule exists exactly once.
#
# The two sides see the repository at different paths — $REPO on the host, /slam-bench
# inside the container — so nothing here may be absolute: preset_launch returns a path
# RELATIVE to the repository root and each caller prepends its own root.

BENCH_DEFAULT_PRESET=default
# 1.0 is the rate at which latency, throughput stability and
# real-time factor to mean anything — the default therefore matches the measurement
# condition, and a faster rate is an explicit opt-in for a quick smoke run.
BENCH_DEFAULT_RATE=1.0
BENCH_DEFAULT_BAG=/bags
# The LiDAR topic our bags carry. A property of the rig, not of any system — every baseline
# is pointed at it by its own config. run_system.sh waits for the system under test to
# subscribe to it before starting playback.
BENCH_LIDAR_TOPIC=/sensing/front/livox/lidar

# preset_launch <system> <preset> -> launch file path, relative to the repo root.
# `default` is the as-shipped upstream configuration; every other preset is a
# standalone file, so a batch can sweep presets with nobody editing configs in between.
preset_launch() {
  if [ "$2" = "$BENCH_DEFAULT_PRESET" ]; then
    echo "configs/launch/$1.launch"
  else
    echo "configs/presets/$1/$2.launch"
  fi
}
