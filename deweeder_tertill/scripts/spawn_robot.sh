#!/usr/bin/env bash
set -e

# USAGE: ./scripts/spawn_robot.sh /path/to/robot.sdf <model_name>
ROBOT_FILE=${1:-""}
NAME=${2:-"mybot"}

if [ -z "$ROBOT_FILE" ]; then
  echo "Usage: $0 /path/to/robot.sdf [model_name]"
  exit 1
fi

echo "Spawning model $NAME from $ROBOT_FILE"

# Prefer ros_gz spawn if available
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  # ros_gz_sim's create (if available)
  echo "Using ros_gz_sim spawn (ros2 run ros_gz_sim create)"
  ros2 run ros_gz_sim create -file "$ROBOT_FILE" -name "$NAME" || true
else
  # fallback to gz CLI (Harmonic)
  if command -v gz >/dev/null 2>&1; then
    echo "Using gz CLI spawn"
    gz model --spawn-file "$ROBOT_FILE" --name "$NAME" || true
  else
    echo "No ros_gz_sim or gz CLI found. Install ros-jazzy-ros-gz or gz simulator."
    exit 2
  fi
fi
