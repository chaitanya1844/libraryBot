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
STOP_PATTERN="async_slam_toolbox_node|nav2_amcl|nav2_map_server|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|smoother_server|lifecycle_manager|rfid_reader_node|shelf_slot_detector_node|task_manager_node"

stop_stack() {
  pkill -9 -f "$STOP_PATTERN" 2>/dev/null || true
  sleep 1
}

case "$MODE" in
  mapping)
    stop_stack
    ros2 launch libbot_mapping mapping.launch.py &
    ;;
  localization)
    stop_stack
    ros2 launch libbot_localization localization.launch.py use_amcl:=true "map:=${MAP_PATH}" &
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
