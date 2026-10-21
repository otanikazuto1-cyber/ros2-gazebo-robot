#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /root/pai_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export DISPLAY="${DISPLAY:-:0}"
MODEL=/root/pai_ws/src/my_robot_control/models/my_robot/model.sdf
BRIDGE=/root/pai_ws/src/my_robot_control/config/bridge.yaml
if [[ ! -f "$MODEL" || ! -f "$BRIDGE" ]]; then
  echo "ERROR: robot model or bridge.yaml missing in /root/pai_ws/src/my_robot_control" >&2
  exit 1
fi
python3 make_world.py
if [[ ! -f fence_policy.npy ]]; then
  echo 'Training Q-learning policy (first run only)...'
  python3 fence_rl.py train
fi
PIDS=()
cleanup() {
  echo 'Stopping demo processes...'
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
# Server, bridge, robot spawn, controller and GUI share the same container.
gz sim -s -r fence_arena.sdf & PIDS+=("$!")
sleep 4
ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$BRIDGE" & PIDS+=("$!")
sleep 2
ros2 run ros_gz_sim create -world arena -file "$MODEL" -name my_robot -z 0.15
sleep 3
python3 fence_rl.py run & PIDS+=("$!")
echo 'Opening Gazebo GUI. Ctrl+C here to stop the demo.'
gz sim -g & PIDS+=("$!")
wait "${PIDS[-1]}"
