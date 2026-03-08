# Deweeder: Autonomous Robot Weed Detection & Removal System

Deweeder is a comprehensive robotics project demonstrating autonomous agricultural robot development with multiple implementation paths: a **Webots-based simulation** for educational purposes and a **ROS 2 (Jazzy)-based control system** for hardware integration. The project showcases computer vision for weed detection, path planning, robotic manipulation, and safety-critical control systems.

## 📋 Project Overview

This repository contains implementations for an autonomous de-weeding robot with:
- **Webots Simulation Environment** - Educational 3D physics simulation with YOLO-based weed detection
- **ROS 2 Integration (deweeder_tertill)** - Production-ready control stack for the Tertill robot platform
- **YOLOv8 Computer Vision** - Real-time weed classification and localization
- **Hardware Abstraction Layer** - Safety-critical control bridging software to hardware

### Key Features
- **Real-time Weed Detection**: OpenCV and YOLOv8-based vision pipeline
- **Autonomous Navigation**: Search patterns and reactive weed-seeking behavior
- **Robotic Arm Control**: Coordinated shoulder/elbow actuation for weed removal
- **Safety Systems**: Watchdog monitoring, heartbeat detection, tilt/contact sensors
- **Hardware Integration**: ROS 2 nodes for sensor fusion, motor control, and tool actuation
- **Simulation & Testing**: Gazebo/Webots for development before hardware deployment

## 🚀 Quick Start

### For Webots Simulation
```bash
# Install dependencies
pip install -r requirements.txt

# Run the simulation
python3 controllers/my_controller/my_controller.py
# or use Webots IDE to load: worlds/Farm of Deweeder.wbt
```

### For ROS 2 (Tertill Robot)
```bash
cd deweeder_tertill
colcon build --symlink-install
source install/setup.bash
ros2 launch weedbot_core robot_core.launch.py
```

### For Full System Simulation (Gazebo)
```bash
./start_simulation.sh
```

## 📁 Repository Structure

```
Deweeder/
├── controllers/              # Webots robot controllers (Python)
│   └── my_controller/        # Main deweeder controller with CV pipeline
├── src/                      # ROS 2 package sources
│   ├── deweeder_robot/       # Webots-integrated ROS 2 nodes
│   └── weedbot_simulation_tertill/  # Tertill simulation package
├── deweeder_tertill/         # Complete ROS 2 Jazzy workspace
│   ├── src/
│   │   ├── weedbot_msgs/     # Custom ROS 2 message definitions
│   │   └── weedbot_core/     # Core robot nodes (state, behavior, safety)
│   ├── scripts/              # Test/debug helper scripts
│   └── tests/                # Unit and integration tests
├── worlds/                   # Webots simulation worlds
├── protos/                   # Webots PROTO definitions (3D models)
├── deweeer_yolo_training.py  # YOLOv8 training pipeline (Colab-ready)
├── Deweeder_Yolov8.ipynb     # Interactive notebook for model training
└── start_simulation.sh       # Multi-terminal simulation launcher

```

## 🤖 Architecture Overview

### Webots Simulation Path
- **Controller**: `controllers/my_controller/my_controller.py` - Implements real-world robot behavior
- **World**: `worlds/Farm of Deweeder.wbt` - Virtual crop field with weeds
- **Vision**: OpenCV-based HSV color detection + contour tracking
- **Navigation**: Zig-zag search pattern with reactive weed approach

### ROS 2 Tertill Control Stack
```
Sensor Input (IMU, encoders, cameras)
        ↓
robot_state_node (sensor fusion → standard ROS messages)
        ↓
behavior_node (decision logic)
        ↓
motion_cmd / tool_cmd
        ↓
motor_controller (PID loops)
        ↓
hal_hw_gpio (Hardware Abstraction Layer)
        ↓
Physical Motors & Tools
```

## 🔧 Core Components

### Controllers
- **my_controller.py** - Main Webots controller implementing weed detection (HSV), navigation (zig-zag), and arm control

### ROS 2 Nodes (deweeder_tertill)
- **robot_state_node** - Converts sensor packets → IMU, Odometry, RobotState messages
- **behavior_node** - High-level planner converting robot state → motion & tool commands
- **safety_node** - Enforces safety constraints (tilt, belly contact, cap detection)
- **motor_controller** - PID-based velocity control with command filtering
- **hal_hw_gpio** - Hardware abstraction with GPIO simulation and motor bridging

### Custom Messages
- `MicroSensorPacket` - Raw sensor data from microcontroller
- `MotionCmd` - Velocity commands for motors
- `ToolCmd` - Tool activation states (trimmer, sprayer, etc.)
- `SafetyCmd` - Safety override signals

## 📊 Development Stages

### Stage 1: Basic Simulation
- Webots environment with crop/weed models
- OpenCV detection pipeline
- Arm IK and movement

### Stage 2: ROS 2 Integration
- Message generation & topic setup
- Individual node development
- Motor controller with PID
- Watchdog/heartbeat safety

### Stage 3: Hardware Ready
- Full pipeline integration (perception → control → actuation)
- Hardware abstraction layer (GPIO/UART bridges)
- Safety validation on real hardware

## 🧪 Testing & Debugging

### Webots Simulation
```bash
# Run controller in Webots IDE
# View: Weed detection visualization (red mask overlay)
# Monitor: Console output for state transitions
```

### ROS 2 System
```bash
# List all nodes
ros2 node list

# Monitor topics in real-time
ros2 topic echo /robot_state
ros2 topic echo /motion_cmd
ros2 topic echo /safety_cmd

# Publish test sensor data
ros2 topic pub /micro/sensor_packet weedbot_msgs/msg/MicroSensorPacket \
  "{ tick_time: 0.1, imu_linear: [0,0,9.8], ... }" -r 10

# Run integration tests
./deweeder_tertill/scripts/watchdog_test.sh
python3 ./deweeder_tertill/scripts/safe_motor_ramp.py
```

## 🧠 Computer Vision

### Training YOLOv8 for Weed Detection
```bash
# Google Colab notebook with full training pipeline
jupyter notebook Deweeder_Yolov8.ipynb

# Or use Python script
python deweeer_yolo_training.py
```

Features:
- Supports multiple YOLO variants (nano, small, medium)
- Automatic dataset validation
- Checkpoint management
- Training metrics & visualization

## 📝 Documentation Files

- **README.md** - This file (project overview)
- **deweeder_tertill/README.md** - Detailed ROS 2 system documentation
- **Small Gardening Robot with Decision-making Watering System.pdf** - Original research paper
- **machines-11-00048-v2.pdf** - Academic publication on the system

## 🔌 Requirements

### Webots Simulation
- Webots 2025a+ (https://cyberbotics.com/)
- Python 3.8+
- OpenCV: `pip install opencv-python`
- NumPy: `pip install numpy`

### ROS 2 Tertill
- ROS 2 Jazzy (Ubuntu 24.04 recommended)
- Gazebo 7+ (for physics simulation)
- Python 3.10+
- colcon build system

### YOLOv8 Training
- PyTorch with CUDA support
- Google Colab (for GPU-accelerated training)
- Dataset of labeled crop/weed images

## 🛠️ Development & Contribution

This is an active educational robotics project. Key areas for contribution:
- Hardware integration (motor drivers, sensor interfaces)
- Path planning algorithms (RRT*, potential fields)
- Advanced CV models (instance segmentation for precise removal)
- Gazebo simulation improvements
- Real-time performance optimization

## 📚 References

- Webots Documentation: https://cyberbotics.com/doc/
- ROS 2 Documentation: https://docs.ros.org/
- YOLOv8 Documentation: https://docs.ultralytics.com/
- Tertill Platform: https://www.tertill.com/

## 👤 Author

Created by Roshan  
Last Updated: December 2025

---

**Status**: Active Development (Milestone 3 - Hardware Integration Ready)
