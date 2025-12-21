#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Publisher for arm control
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/shoulder_controller/joint_trajectory', 
            10
        )
        
        # Subscriber for weed detection
        self.weed_sub = self.create_subscription(
            Bool,
            '/weed_detected',
            self.weed_callback,
            10
        )
        
        self.get_logger().info('🦾 Arm Controller Started!')
        
    def move_arm_to_position(self, shoulder_pos, elbow_pos):
        """Move arm to specified positions"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['shoulder_joint', 'elbow_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [shoulder_pos, elbow_pos]
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()
        
        trajectory.points.append(point)
        self.arm_pub.publish(trajectory)
        
        self.get_logger().info(f'🎯 Moving arm: shoulder={shoulder_pos:.2f}, elbow={elbow_pos:.2f}')
        
    def perform_weeding_action(self):
        """Perform weeding sequence"""
        self.get_logger().info('🛠️ Starting weeding sequence...')
        
        # Move to approach position
        self.move_arm_to_position(0.0, 0.8)
        time.sleep(2)
        
        # Spray forward
        self.move_arm_to_position(0.0, 1.2)
        self.get_logger().info('💦 Spraying weed...')
        time.sleep(1)
        
        # Return
        self.move_arm_to_position(0.0, 0.8)
        time.sleep(1)
        
        # Reset to travel position
        self.move_arm_to_position(0.0, 0.5)
        self.get_logger().info('✅ Weeding complete!')
        
    def weed_callback(self, msg):
        """Handle weed detection messages"""
        if msg.data:
            self.get_logger().info('🚨 Weed detected - preparing arm for weeding...')
            # Wait a bit for robot to get closer, then perform weeding
            time.sleep(3)
            self.perform_weeding_action()

def main():
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
