# Deweeder Robot Simulation

Deweeder is an educational simulation of an autonomous de-weeding robot built with Webots and Python. The project demonstrates basic mobile-robot navigation, simple computer-vision weed detection using OpenCV, and a small robotic arm for weed removal in a virtual farm environment. The repository also includes an optional ROS integration package in `deweeder_ros/` for running the simulation and controllers within a ROS environment.

## deweeder_ros — ROS Integration (detailed)

This section explains how to use the deweeder_ros package to run the Deweeder simulation with ROS. The `deweeder_ros/` folder is intended to provide ROS-compatible nodes, message/topic bridges, and launch files that connect Webots simulation components (camera, motors, arm) to ROS topics and services.

IMPORTANT: The exact files, launch names, and dependencies inside `deweeder_ros/` determine whether it targets ROS1 (Noetic, Melodic) or ROS2 (Humble, Foxy). Check `deweeder_ros/` for a README or the launch/ and package manifests (package.xml / CMakeLists.txt) to confirm target ROS distribution. The instructions below cover typical workflows for both ROS1 and ROS2.

### Common prerequisites

- A supported Linux distribution for your ROS version (Ubuntu 20.04 for ROS1 Noetic; Ubuntu 22.04 for ROS2 Humble is common).
- Webots installed (https://cyberbotics.com/) and in your PATH.
- Python 3 and project Python deps: `pip install -r requirements.txt` (or `pip install opencv-python numpy`).
- ROS (see sections below).

### Check the package layout

Before proceeding, inspect the `deweeder_ros/` folder to see what is included. Typical structure to look for:

````markdown
deweeder_ros/
├── package.xml          # ROS package manifest (ROS1/ROS2)
├── CMakeLists.txt       # build instructions for ROS1 or ROS2
├── launch/              # ROS1 launch files (.launch) or ROS2 launch (Python)
├── config/              # any configuration files (params, YAML)
├── scripts/             # ROS nodes (Python) or executables
└── README.md            # package-specific instructions (if present)
````

If `package.xml` and `CMakeLists.txt` use `ament_*` macros, the package is likely ROS2. If they use `catkin` macros, it is ROS1.

---

## ROS1 (Noetic) — Example setup and run

Use these steps if `deweeder_ros` is a catkin (ROS1) package. Adjust distro names as needed.

1) Install ROS1 Noetic and dependencies (follow official ROS install instructions):

   sudo apt update
   sudo apt install ros-noetic-desktop-full python3-rosdep python3-catkin-tools
   sudo rosdep init || true
   rosdep update

2) Create or use a catkin workspace and clone the repo into src:

   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/codzofrosh/Deweeder.git
   cd ~/catkin_ws

3) Install ROS dependencies declared in `deweeder_ros/package.xml` (example):

   rosdep install --from-paths src -i -y --rosdistro noetic

4) Build the workspace:

   catkin_make

5) Source the workspace and ROS environment:

   source /opt/ros/noetic/setup.bash
   source devel/setup.bash

6) Run the Webots + ROS launch (example):

   roslaunch deweeder_ros deweeder_webots.launch

Replace `deweeder_webots.launch` with the actual launch file name inside `deweeder_ros/launch/`. Typical launch setups will start Webots in paused mode or connect to a running Webots instance via the webots_ros package (webots_ros or webots_ros2 bridge may be required).

7) In other terminals you can inspect topics and nodes:

   rostopic list
   rosnode list
   rqt_image_view /camera/image_raw

---

## ROS2 (Humble/Foxy) — Example setup and run

If `deweeder_ros` targets ROS2, follow these steps (example uses ROS2 Humble):

1) Install ROS2 and dependencies (see ROS2 installation guide).

2) Create or use a ROS2 workspace and clone repo into src:

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/codzofrosh/Deweeder.git
   cd ~/ros2_ws

3) Install dependencies listed in `deweeder_ros/package.xml` or `package.xml` and any ROS2-specific Python packages:

   rosdep install --from-paths src -i -y --rosdistro humble

4) Build with colcon:

   source /opt/ros/humble/setup.bash
   colcon build --symlink-install

5) Source the workspace:

   source install/setup.bash

6) Run a launch file (example):

   ros2 launch deweeder_ros deweeder_webots_launch.py

Replace `deweeder_webots_launch.py` with the actual ROS2 launch file name. If the package uses the `webots_ros2` interface, the launch may start Webots and the ROS2 nodes together.

7) Inspect topics and images:

   ros2 topic list
   ros2 run rqt_image_view rqt_image_view --topic /camera/image_raw

---

## Running Webots and ROS together (tips)

- Option A: Launch Webots from the ROS launch file (common when launch includes a Webots node or uses the webots_ros bridge).
- Option B: Start Webots separately and have ROS nodes connect to it via ROS topics/services. In this case, start Webots, load the `worlds/Farm of Deweeder.wbt` world, start the robot controller in Webots as a ROS-enabled controller, and then source your ROS workspace and run any ROS-specific nodes.
- Ensure the Webots simulator and ROS are using compatible bridges (e.g., `webots_ros` for ROS1, `webots_ros2` for ROS2).

## Typical topics and services to check

- /camera/image_raw (sensor_msgs/Image) — robot camera feed
- /cmd_vel (geometry_msgs/Twist) — mobile base velocity commands
- /arm_controller/command (trajectory_msgs/JointTrajectory) — arm control (varies by controller)
- /weed_detected (std_msgs/Bool or custom message) — detection flag (if implemented)

Check the `deweeder_ros` code to confirm exact topic names and message types.

## Troubleshooting

- If Webots does not start from the launch file, try launching Webots manually and connect nodes to the running simulator.
- If topics are missing, confirm the robot controller in Webots is the ROS-enabled controller (it may be under `controllers/` and call ROS APIs).
- If images are empty or black, verify camera resolution and Webots camera sampling settings.

## If you want, I can: 
- Create a `deweeder_ros/README.md` with precise commands tailored to the package contents (tell me which ROS distro you target or I can inspect `deweeder_ros/` to detect it),
- Add example launch files or a minimal bridge if `deweeder_ros` is missing them.

---

*Created by Roshan*
