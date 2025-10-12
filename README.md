# Deweeder Robot Simulation

This project simulates an autonomous de-weeding robot using Webots and Python. The robot navigates a virtual farm, detects weeds using computer vision, and removes them with a robotic arm.

## Features

- **Webots World**: Realistic farm environment with crops and weeds.
- **E-puck Robot**: Mobile robot base with camera and wheel motors.
- **Robotic Arm**: Two-joint arm for precise weed removal.
- **Computer Vision**: Weed detection using OpenCV (color-based).
- **Autonomous Navigation**: Zig-zag search, weed approach, and removal sequence.

## Directory Structure

```
Deweeder/
├── controllers/
│   └── my_controller/
│       └── my_controller.py      # Python controller for robot and arm
├── worlds/
│   └── Farm of Deweeder.wbt      # Webots world file (farm, crops, weeds, robot)
```

## Getting Started

1. **Requirements**
   - [Webots](https://cyberbotics.com/)
   - Python 3
   - OpenCV (`pip install opencv-python`)
   - NumPy (`pip install numpy`)

2. **Usage**
   - Open `Farm of Deweeder.wbt` in Webots.
   - Ensure `my_controller.py` is set as the controller for the robot.
   - Run the simulation. The robot will search for weeds and remove them autonomously.

## Customization

- **World File**: Edit `Farm of Deweeder.wbt` to change crop/weed layout.
- **Controller**: Modify `my_controller.py` for different detection or movement strategies.

## License

This project is for educational and research purposes.

---
*Created by [Your Name]*