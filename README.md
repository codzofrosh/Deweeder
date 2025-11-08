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
name=README.md url=https://github.com/codzofrosh/Deweeder/blob/ff58b4dc94332574aa0736e454c6bff0b8a9b080/README.md
Deweeder/
├── controllers/
│   └── my_controller/
│       └── my_controller.py      # Python controller for robot and arm
├── worlds/
│   └── Farm of Deweeder.wbt      # Webots world file (farm, crops, weeds, robot)
````

## Requirements

- Webots (stable release) — https://cyberbotics.com/
- Python 3.8+ (3.10 recommended)
- OpenCV: pip install opencv-python
- NumPy: pip install numpy

## Quick Start

1. Install Webots and confirm it runs on your system.
2. Install Python dependencies: pip install -r requirements.txt or pip install opencv-python numpy
3. Open `worlds/Farm of Deweender.wbt` in Webots.
4. Ensure the robot’s controller is set to `my_controller` (or `my_controller.py`).
5. Run the simulation. The controller will search for weeds, approach, and remove them.

## Customization

- World: Edit `worlds/Farm of Deweeder.wbt` to change plant/weeds layout and environment settings.
- Controller: Modify `controllers/my_controller/my_controller.py` to change detection thresholds, movement strategy, or arm behavior.

## Notes & Limitations

- The weed detector is a simple color-based approach and may need tuning for different lighting or weed appearances.
- This project is intended for learning and research; it is not production-ready.

## Contributing

Contributions and suggestions are welcome. Open an issue or submit a pull request with improvements.

## License

This project is provided for educational and research purposes.

---

*Created by Roshan*