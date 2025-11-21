#!/usr/bin/env bash
set -eu

# temporarily relax nounset while sourcing ROS overlay (some setup scripts
# reference variables that may be undefined when -u is active)
set +u
source install/setup.bash
set -u

# auto-detect a usable RMW implementation (prefer cyclonedds if installed)
if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
  echo "Using RMW_IMPLEMENTATION from env: $RMW_IMPLEMENTATION"
else
  # check common locations for the Cyclone RMW library
  if [ -f "/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so" ] || ldconfig -p 2>/dev/null | grep -q librmw_cyclonedds_cpp; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  elif [ -f "/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so" ] || ldconfig -p 2>/dev/null | grep -q librmw_fastrtps_cpp; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  else
    # leave default (ros2 will pick whatever is available)
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}
  fi
  echo "Auto-selected RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
fi


echo "Starting core WITHOUT behavior..."
ros2 launch weedbot_core robot_core.launch.py use_behavior:=False > /tmp/launch_core.log 2>&1 &
LAUNCH_PID=$!
sleep 2

echo "Start HAL (simulate)"
ros2 run weedbot_core hal_hw_gpio --ros-args --log-level INFO > /tmp/hal.log 2>&1 &
HAL_PID=$!
sleep 1

echo "Start motor_controller"
ros2 run weedbot_core motor_controller --ros-args --log-level INFO > /tmp/motor.log 2>&1 &
MC_PID=$!
sleep 1

echo "Start encoder publisher (sim)"
ros2 topic pub /micro/sensor_packet weedbot_msgs/msg/MicroSensorPacket '{"tick_time":0.05,"wheel_encoder":[10,10,10,10],"imu_linear":[0.0,0.0,9.8],"imu_angular":[0.0,0.0,0.0],"motor_currents":[0,0,0,0,0,0],"cap_front":false,"cap_left":false,"cap_right":false,"cap_rear":false,"belly_contact":false}' -r 20 > /dev/null 2>&1 &
ENC_PID=$!

sleep 1

echo "Start behavior_node (to provide heartbeat)"
ros2 run weedbot_core behavior_node --ros-args --log-level INFO > /tmp/behavior.log 2>&1 &
BEHAV_PID=$!
sleep 1

echo "Run ramp script"
python3 scripts/safe_motor_ramp.py > /tmp/ramp.log 2>&1 &
RAMP_PID=$!

# give system time to operate
sleep 2

echo "Killing behavior_node to trigger heartbeat loss..."
kill $BEHAV_PID || true

# wait heartbeat timeout (default 0.5s) + margin
sleep 1.2

echo "Check HAL diag for zeroing message"
if grep -q "Heartbeat stale -> zeroing motion outputs" /tmp/hal.log; then
    echo "WATCHDOG: HAL detected stale heartbeat (PASS)"
else
    echo "WATCHDOG: HAL did NOT detect stale heartbeat (FAIL)"
    echo "---- HAL log tail ----"
    tail -n 100 /tmp/hal.log || true
fi

# cleanup
kill $RAMP_PID $ENC_PID $MC_PID $HAL_PID $LAUNCH_PID 2>/dev/null || true
echo "Done"
