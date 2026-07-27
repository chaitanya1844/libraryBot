#!/bin/bash
# Stops whichever localization/navigation stage is currently running and
# starts the requested one, so the control panel's Mode buttons can be
# used as a straightforward Map -> Localize -> Navigate workflow without
# two stages fighting over the map->odom TF.
#
# Usage: switch_mode.sh <mapping|localization|navigation|stop> [map_yaml_path]

set -u
MODE="${1:-}"
MAP_PATH="${2:-$HOME/libraryBot/ros2_ws/src/libbot_mapping/maps/library.yaml}"

# Everything that could own map->odom, plus the full Nav2 autonomy stack
# that sits on top of localization. Matches the same process-name
# patterns used throughout this repo's own testing/cleanup.
STOP_PATTERN="async_slam_toolbox_node|nav2_amcl|nav2_map_server|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|smoother_server|lifecycle_manager|task_manager_node|fastlio_mapping|icp_odometry|lidar_3d_velodyne_adapter|fastlio_map_odom_bridge"

# FAST-LIO2 replaced AMCL as the localization source (see
# libbot_localization/launch/localization_fastlio.launch.py) -- it lives
# in a separate ~/fastlio_ws overlay workspace since it isn't packaged
# for apt, so it has to be sourced here explicitly, on top of whatever
# ROS/libraryBot environment this script inherited.
FASTLIO_WS_SETUP="$HOME/fastlio_ws/install/setup.bash"

stop_stack() {
  pkill -9 -f "$STOP_PATTERN" 2>/dev/null || true
  sleep 1
}

# Both mapping and localization run FAST-LIO2 (mapping.launch.py now
# includes localization_3d.launch.py -- slam_toolbox was removed, see
# that launch file's docstring), so both need the overlay sourced.
source_fastlio_ws() {
  if [ -f "$FASTLIO_WS_SETUP" ]; then
    # colcon-generated setup scripts reference $COLCON_TRACE without a
    # default and abort under `set -u` ("unbound variable") -- this
    # silently killed the whole script before it ever reached the
    # `ros2 launch` below, with no error visible anywhere except this
    # script's own stderr (which QProcess::startDetached from the
    # panel discards). Relax nounset only around the source calls.
    set +u
    source "$FASTLIO_WS_SETUP"
    # fastlio_ws's own setup.bash chains back to whatever *other*
    # overlay workspaces were sourced on this machine when it was
    # built (unrelated other robot projects), not this repo's own
    # ros2_ws -- re-source libraryBot's install on top so it's the
    # highest-priority overlay again, regardless of what fastlio_ws's
    # own chain pulled in.
    source "$HOME/libraryBot/ros2_ws/install/setup.bash"
    set -u
  else
    echo "switch_mode.sh: $FASTLIO_WS_SETUP not found -- see README '3D localization' to build it" >&2
  fi
}

case "$MODE" in
  mapping)
    stop_stack
    source_fastlio_ws
    ros2 launch libbot_mapping mapping.launch.py &
    ;;
  localization)
    stop_stack
    source_fastlio_ws
    ros2 launch libbot_localization localization_fastlio.launch.py "map:=${MAP_PATH}" &
    ;;
  navigation)
    # Additive on top of an already-running localization stage -- does
    # not stop anything, since Nav2 needs AMCL's map->odom to already
    # be flowing.
    ros2 launch libbot_bringup autonomy.launch.py &
    ;;
  stop)
    stop_stack
    ;;
  *)
    echo "usage: switch_mode.sh <mapping|localization|navigation|stop> [map_yaml_path]" >&2
    exit 1
    ;;
esac
