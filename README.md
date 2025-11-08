# Deweeder Robot Simulation

Deweeder is an educational simulation of an autonomous de-weeding robot built with Webots and Python. The project demonstrates basic mobile-robot navigation, simple computer-vision weed detection using OpenCV, and a small robotic arm for weed removal in a virtual farm environment.

## Key Features

- Webots world simulating a small farm with crops and weeds.
- E-puck mobile base equipped with a forward-facing camera and wheel motors.
- Two-joint robotic arm for precise weed removal.
- Computer vision using OpenCV (color-based detection) and NumPy for processing.
- Autonomous behaviors: systematic search (zig-zag), approach, and removal sequence.

## Repository Layout

````markdown
name=README.md url=https://github.com/codzofrosh/Deweeder/blob/main/README.md
Deweeder/
├── controllers/
│   └── my_controller/
│       └── my_controller.py      # Python controller for robot and arm
├── worlds/
│   └── Farm of Deweeder.wbt      # Webots world file (farm, crops, weeds, robot)
├── deweeder_ros/                 # (optional) ROS package for integration with ROS/ROS2
````

## ROS Integration

The repository includes an optional deweeder_ros folder (if present) that contains ROS-compatible nodes, launch files, and configuration to run the simulation components under a ROS environment. This can be useful for:

- Integrating the controller with ROS topics/services for perception and control.
- Testing ROS-based planning or mapping stacks with the simulated robot.

Note: ROS package contents and compatibility (ROS1 vs ROS2) depend on the files in deweeder_ros; check that folder for instructions and any required dependencies.

## Requirements

- Webots (stable release) — https://cyberbotics.com/
- Python 3.8+ (3.10 recommended)
- OpenCV: pip install opencv-python
- NumPy: pip install numpy

## Quick Start

1. Install Webots and confirm it runs on your system.
2. Install Python dependencies: pip install -r requirements.txt or pip install opencv-python numpy
3. Open `worlds/Farm of Deweeder.wbt` in Webots.
4. Ensure the robot’s controller is set to `my_controller` (or `my_controller.py`).
5. If using ROS, inspect `deweeder_ros/` for launch files and follow its README or instructions.
6. Run the simulation. The controller will search for weeds, approach, and remove them.

## Customization

- World: Edit `worlds/Farm of Deweeder.wbt` to change plant/weeds layout and environment settings.
- Controller: Modify `controllers/my_controller/my_controller.py` to change detection thresholds, movement strategy, or arm behavior.
- ROS: If `deweeder_ros/` exists, update its launch/config files to connect to your ROS setup.

## Notes & Limitations

- The weed detector is a simple color-based approach and may need tuning for different lighting or weed appearances.
- This project is intended for learning and research; it is not production-ready.

## Contributing

Contributions and suggestions are welcome. Open an issue or submit a pull request with improvements.

## License

This project is provided for educational and research purposes.

---

*Created by Roshan*