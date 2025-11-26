#!/usr/bin/env bash
set -euo pipefail
# hw_smoke_test.sh
# Simple smoke test that launches the core stack in simulation mode,
# waits for /hal/ready and publishes a small sequence of motion commands.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/install/setup.bash" || { echo "source install/setup.bash failed"; exit 1; }

echo "Starting robot_core in simulation (hw_mode=False)..."
ros2 launch weedbot_core robot_core.launch.py hw_mode:=False hw_backend:=mock > /tmp/hw_smoke_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo "Cleaning up..."
  kill ${LAUNCH_PID} 2>/dev/null || true
  wait ${LAUNCH_PID} 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting for /hal/ready..."
READY=false
for i in $(seq 1 15); do
  # try to read once; timeout short
  OUT=$(ros2 topic echo -n 1 /hal/ready 2>/dev/null || true)
  if echo "$OUT" | grep -q "data: true"; then
    READY=true
    break
  fi
  sleep 1
done

if [ "$READY" != "true" ]; then
  echo "HAL NOT ready - aborting"
  cat /tmp/hw_smoke_launch.log || true
  exit 2
fi
echo "HAL ready confirmed."

# publish a safe low-power motion sequence using MotionCmd
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from weedbot_msgs.msg import MotionCmd
rclpy.init()
n = Node("smoke_publisher")
pub = n.create_publisher(MotionCmd, "/motion_cmd", 10)
# gentle sweep: 0.0 -> 0.3 -> 0.0
for duty in [0.0, 0.1, 0.2, 0.3, 0.0]:
    m = MotionCmd()
    m.linear_x = float(duty)
    m.angular_z = 0.0
    pub.publish(m)
    n.get_logger().info(f"published motion duty={duty}")
    rclpy.spin_once(n, timeout_sec=0.5)
n.destroy_node()
rclpy.shutdown()
PY

echo "Smoke test finished successfully."
exit 0
