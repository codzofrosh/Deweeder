# Deweeder Tertill-based Robot (WeedBot) (Tested on ROS2 Jazzy)
Document Structure
deweeder_tertill/               # workspace root
├── src/
│   ├── weedbot_msgs/           # custom messages (MicroSensorPacket, MotionCmd, etc.)
│   │   ├── msg/
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── weedbot_core/           # nodes & launch
│       ├── weedbot_core/
│       │   ├── robot_state_node.py
│       │   ├── behavior_node.py
│       │   └── safety_node.py
│       ├── launch/
│       │   └── robot_core.launch.py
│       └── package.xml / setup.py
├── scripts/                    # helper publishers, bridges, test scripts
│   ├── pub_sensor.py
│   ├── pub_motion.py
│   └── sensor_to_state_bridge.py
├── docs/
│   ├── STRUCTURE.md            # this file (file tree + descriptions)
│   └── NODE_DIAGRAM.dot        # topic/node graph (Graphviz)
├── README.md
├── .gitignore
└── tests/

Short file purpose:

weedbot_msgs: message IDL files are the single source of truth for comms.

weedbot_core/weedbot_core/*.py: runtime nodes (robot state, behavior/planner, safety).

launch/robot_core.launch.py: launches your node set.

scripts/*.py: small test harnesses to publish or bridge topics for manual tests.

docs/NODE_DIAGRAM.dot: a DAG you can render into PNG to show the comm graph.

Node descriptions

robot_state_node

Subscribes: /micro/sensor_packet (weedbot_msgs/msg/MicroSensorPacket)

Publishes: /imu (sensor_msgs/Imu), /odom (nav_msgs/Odometry), /robot_state (geometry_msgs/Twist)

Role: convert raw micro sensor bundle into standard ROS messages and a simplified robot state (Twist). Also handles type coercions and filtering.

behavior_node

Subscribes: /robot_state (geometry_msgs/Twist)

Publishes: /motion_cmd (weedbot_msgs/MotionCmd), /tool_cmd (weedbot_msgs/ToolCmd)

Role: decision logic/planner. Converts state to motion/tool commands. Uses parameters for thresholds and scaling.

safety_node

Subscribes: /robot_state (geometry_msgs/Twist)

Publishes: /safety_cmd (weedbot_msgs/SafetyCmd)

Role: enforces safety rules (tilt, belly contact, cap detection). Publishes a discrete safety state that behavior and HAL must honor.


Short description

## Quickstart
1. Build: `colcon build --symlink-install`/
2. Source: `source install/setup.bash`
3. Launch: `ros2 launch weedbot_core robot_core.launch.py`

# launch full robot stack
ros2 launch weedbot_core robot_core.launch.py

# run individual nodes in foreground (useful for debug)
ros2 run weedbot_core robot_state_node --ros-args --log-level DEBUG
ros2 run weedbot_core behavior_node --ros-args --log-level DEBUG
ros2 run weedbot_core safety_node --ros-args --log-level DEBUG

**Inspect topics & messages**

ros2 node list
ros2 node info /behavior_node
ros2 topic list -t
ros2 topic info /micro/sensor_packet
ros2 interface show weedbot_msgs/msg/MicroSensorPacket
ros2 topic echo /robot_state --once
ros2 topic echo /motion_cmd --once
ros2 topic echo /tool_cmd --once
ros2 topic echo /safety_cmd --once

# publish a single test MicroSensorPacket
ros2 topic pub /micro/sensor_packet weedbot_msgs/msg/MicroSensorPacket \
"{ tick_time: 0.1, imu_linear: [0.0,0.0,9.8], imu_angular: [0.0,0.0,0.0], motor_currents: [0,0,0,0,0,0], wheel_encoder: [0,0,0,0], cap_front: false, cap_left: false, cap_right: false, cap_rear: false, belly_contact: false }" -1

# publish robot_state to force behavior
ros2 topic pub /robot_state geometry_msgs/msg/Twist "{ linear: { x: 0.4 }, angular: { z: 0.0 } }" -r 1

# publish motion/tool commands directly for HAL testing
ros2 topic pub /motion_cmd weedbot_msgs/msg/MotionCmd "{ linear_x: 0.2, angular_z: 0.0 }" -r 1
ros2 topic pub /tool_cmd weedbot_msgs/msg/ToolCmd "{ front_trimmer_on: true, belly_trimmer_mode: 1 }" -r 1

# show topic publishers/subscribers with types and QoS
ros2 topic info /topic_name

# increase node log level
ros2 run <pkg> <node> --ros-args --log-level DEBUG

# source installed overlay
source install/setup.bash

## How to test
(put helpful `ros2 topic pub` & `ros2 topic echo` examples)

**Milestone 2**
Milestone 2 focuses on stabilizing the ROS 2 system, repairing message generation issues, bringing up all major nodes, connecting the full perception→behavior→motor→HAL pipeline, and verifying safety + watchdog behavior in simulation.

This section documents everything needed to run, debug and test the system as of Milestone 2.
| Node                       | Purpose                                                    | Key Topics                                               |
| -------------------------- | ---------------------------------------------------------- | -------------------------------------------------------- |
| **robot_state_node**       | Converts MicroSensorPacket → IMU + Odometry + RobotState   | `/imu`, `/odom`, `/robot_state`                          |
| **behavior_node**          | High-level behavior → MotionCmd + ToolCmd                  | `/robot_state`, `/motion_cmd`, `/tool_cmd`               |
| **safety_node**            | Ensures system safety + publishes SafetyCmd                | `/robot_state`, `/safety_cmd`                            |
| **motor_controller**       | PID motor control + command filtering + heartbeat tracking | `/motion_cmd`, `/micro/sensor_packet`                    |
| **hal_node / hal_hw_gpio** | Hardware abstraction layer                                 | `/motion_cmd`, `/tool_cmd`, `/safety_cmd` → GPIO actions |

deweeder_tertill/
│
├── src/
│   ├── weedbot_msgs/          # All custom ROS2 interfaces (msg/)
│   │   ├── msg/
│   │   │   ├── MicroSensorPacket.msg
│   │   │   ├── MotionCmd.msg
│   │   │   ├── SafetyCmd.msg
│   │   │   └── ToolCmd.msg
│   │   └── CMakeLists.txt & package.xml (patched for Jazzy)
│   │
│   ├── weedbot_core/          # All functional robot nodes
│   │   ├── behavior_node.py
│   │   ├── robot_state_node.py
│   │   ├── safety_node.py
│   │   ├── motor_controller.py
│   │   ├── hal_node.py
│   │   ├── hal_hw_gpio.py     # HAL GPIO simulation / hardware bridge
│   │   └── launch/robot_core.launch.py
│
├── scripts/
│   ├── safe_motor_ramp.py     # Motor ramp test
│   ├── watchdog_test.sh       # Auto heartbeat/timeout test
│   └── encoder_pub_sim.py     # Fake wheel encoder publisher
│
└── install/ build/ log/       # Auto-generated by colcon
1. Build
colcon build --symlink-install
source install/setup.bash

2. Launch the Core System
ros2 launch weedbot_core robot_core.launch.py


This starts:

robot_state_node

behavior_node

safety_node

🧪 Testing & Debug Commands

Below are all verified milestone-2 test commands.

A. Publish Simulated Sensor Data
ros2 topic pub /micro/sensor_packet weedbot_msgs/msg/MicroSensorPacket \
"{ tick_time: 0.1,
   wheel_encoder:[0,0,0,0],
   imu_linear:[0,0,9.8],
   imu_angular:[0,0,0],
   motor_currents:[0,0,0,0,0,0],
   cap_front:false, cap_left:false, cap_right:false, cap_rear:false,
   belly_contact:false }" -r 10

B. Check Outputs from robot_state_node
ros2 topic echo /imu --once
ros2 topic echo /odom --once
ros2 topic echo /robot_state

C. Test Behavior Node Command Generation

Inject robot state:

ros2 topic pub /robot_state geometry_msgs/msg/Twist \
"{ linear:{x:0.4}, angular:{z:0.0} }" -r 1


Check outputs:

ros2 topic echo /motion_cmd --once
ros2 topic echo /tool_cmd --once

D. Check Safety Node
ros2 topic echo /safety_cmd

E. HAL Simulation (GPIO / Motor outputs)
Start HAL:
ros2 run weedbot_core hal_hw_gpio -- --hw --ros-args --log-level INFO


You will see actions like:

apply_tool -> front=True mode=1 safety=0

F. Motor Controller Node + PID Parameters

Start:

ros2 run weedbot_core motor_controller --ros-args --log-level INFO


Set parameters:

ros2 param set /motor_controller kp 1.5
ros2 param set /motor_controller ki 0.2
ros2 param set /motor_controller kd 0.02
ros2 param set /motor_controller max_pwm 200.0


Verify:

ros2 param get /motor_controller kp

G. Motor Ramp Test (safe_motor_ramp.py)
python3 scripts/safe_motor_ramp.py


Expected:

publishing 0.0
publishing 0.1
publishing 0.2
...

H. Watchdog Test (Heartbeat timeout)
./scripts/watchdog_test.sh


Expected:

WATCHDOG: HAL detected stale heartbeat (PASS)


Safety verified.

📘 Milestone 2 Achievement Summary

✔ Fixed ROS2 interface generation under Jazzy
✔ All custom messages generated correctly
✔ robot_state_node producing IMU + Odom + State
✔ behavior_node generating motion + tool commands
✔ safety_node enforcing safe conditions
✔ motor_controller with PID + parameter server working
✔ HAL (hardware abstraction layer) responding in simulation
✔ safe ramp test validated motor command flow
✔ watchdog test passed (system halts when behavior_node dies)

The system now represents a fully functional simulated robot pipeline ready for hardware integration.

**Milestone 3**
ROS 2 (Jazzy) workspace: weedbot_core, weedbot_msgs

Custom messages generated and importable: weedbot_msgs/msg/{MicroSensorPacket,MotionCmd,ToolCmd,SafetyCmd}

Nodes implemented and running:

robot_state_node — reads MicroSensorPacket, publishes /imu, /odom, /robot_state

behavior_node — reads /robot_state, publishes /motion_cmd and /tool_cmd

safety_node — monitors /robot_state, publishes /safety_cmd

motor_controller — subscribes /motion_cmd, interfaces with HAL

hal_hw_gpio — HAL driver (mock mode by default), publishes /hal/ready and /hal/diag

Watchdog & heartbeat test passing (stale heartbeat detection)

Protective gating and safety overrides implemented in HAL/motor controller (simulation)

How to run (commands used for testing)

# build + source
colcon build --symlink-install
COLCON_TRACE=0 source install/setup.bash

# launch whole system (simulation)
ros2 launch weedbot_core robot_core.launch.py hw_mode:=False

# verify nodes
ros2 node list

# verify HAL readiness and diag
ros2 topic echo /hal/ready --once
ros2 topic echo /hal/diag --once

# publish test sensor packet (simulator)
ros2 topic pub /micro/sensor_packet weedbot_msgs/msg/MicroSensorPacket \
  "{ tick_time: 0.1, imu_linear: [0.0,0.0,9.8], imu_angular: [0.0,0.0,0.0], \
     motor_currents: [0,0,0,0,0,0], wheel_encoder: [0,0,0,0], \
     cap_front: false, cap_left: false, cap_right: false, cap_rear: false, \
     belly_contact: false }" -r 1

# publish motion command
ros2 topic pub /motion_cmd weedbot_msgs/msg/MotionCmd "{ linear_x: 0.2, angular_z: 0.0 }" -1

# view motor/hal diagnostics
ros2 topic echo /hal/diag --once


Important dev-debug commands used

ros2 node list

ros2 node info <node>

ros2 topic list -t

ros2 topic echo <topic> --once

ros2 topic pub <topic> <msg> <payload> -r N or -1

./scripts/watchdog_test.sh (integration smoke test)

python3 scripts/safe_motor_ramp.py (ramp test)

ros2 param set/get <node> <param> (tuning PID)