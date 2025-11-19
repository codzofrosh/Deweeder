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
