#!/bin/bash

echo "🚀 Starting De-weeder Simulation..."

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/deweeder_ws/install/setup.bash

# Get package directory
PKG_DIR=~/deweeder_ws/src/deweeder_robot

echo "📁 Package directory: $PKG_DIR"

# Terminal 1: Start Gazebo with world
echo "🌍 Starting Gazebo with crop field world..."
gnome-terminal --title="Gazebo" -- bash -c "gz sim -r $PKG_DIR/worlds/crop_field.world; exec bash"

# Wait for Gazebo to start
sleep 7

# Terminal 2: Start robot state publishers
echo "🤖 Starting robot state publishers..."
gnome-terminal --title="Robot State" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/deweeder_ws/install/setup.bash; ros2 run robot_state_publisher robot_state_publisher $PKG_DIR/urdf/deweeder_robot.urdf; exec bash"

sleep 2

# Terminal 3: Start joint state publisher
gnome-terminal --title="Joint State" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/deweeder_ws/install/setup.bash; ros2 run joint_state_publisher joint_state_publisher; exec bash"

sleep 2

# Terminal 4: Spawn robot in Gazebo
echo "🎯 Spawning robot in Gazebo..."
gnome-terminal --title="Spawn Robot" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/deweeder_ws/install/setup.bash; gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req 'sdf_filename: \"$PKG_DIR/urdf/deweeder_robot.urdf\"'; exec bash"

sleep 3

# Terminal 5: Start weed detector
echo "🌱 Starting weed detector..."
gnome-terminal --title="Weed Detector" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/deweeder_ws/install/setup.bash; ros2 run deweeder_robot weed_detector; exec bash"

sleep 2

# Terminal 6: Start arm controller
echo "🦾 Starting arm controller..."
gnome-terminal --title="Arm Controller" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/deweeder_ws/install/setup.bash; ros2 run deweeder_robot arm_controller; exec bash"

echo "✅ All components started!"
echo "📋 Open terminals:"
echo "   - Gazebo (3D simulation)"
echo "   - Robot State (TF transforms)" 
echo "   - Joint State (Joint positions)"
echo "   - Weed Detector (Camera processing)"
echo "   - Arm Controller (Arm movements)"

# Keep the script running
wait
