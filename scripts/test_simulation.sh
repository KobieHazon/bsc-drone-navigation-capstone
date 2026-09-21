#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
source /opt/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export ROS_HOME=/tmp/drone-ros
roscore > /tmp/drone-roscore.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
for attempt in {1..60}; do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.5
done
rosparam list >/dev/null
timeout 90s xvfb-run -a python2 tests/test_simulation.py
