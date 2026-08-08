#!/bin/bash
# Wraps `ros2 launch libbot_bringup simulation.launch.py` so Ctrl-C
# always tears the whole stack down, not just this launch's own
# children.
#
# `ros2 launch`'s SIGINT handling only reaches processes it directly
# started. It never reaches gzserver reliably (a known launch/gzserver
# quirk -- gzserver forks/execs internally in a way that doesn't
# propagate SIGINT cleanly) and it never reaches the mapping/
# localization/navigation stack switch_mode.sh starts, since the panel
# (or the CLI) launches each of those as its own separate, detached
# `ros2 launch` -- not a child of this one. A launch-internal
# OnShutdown handler was tried here before; verified (2026-08-08) that
# `launch` does not actually run new processes from on_shutdown actions
# on this Humble version, so it silently never fired. A plain bash
# `trap` on this wrapper script is the reliable version of the same
# idea -- see simulation.launch.py's own comment for the full story.
#
# Usage: same arguments as `ros2 launch libbot_bringup simulation.launch.py`
#   run_simulation.sh
#   run_simulation.sh headless:=true
#   run_simulation.sh rviz:=false

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cleanup() {
  echo "run_simulation.sh: cleaning up (stopping any mapping/localization/navigation stack, gzserver, rviz2)..." >&2
  "$SCRIPT_DIR/switch_mode.sh" stop
  pkill -9 -f 'gzserver|gzclient|rviz2' 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch libbot_bringup simulation.launch.py "$@"
