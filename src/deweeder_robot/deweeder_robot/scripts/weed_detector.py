#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class WeedDetector(Node):
    def __init__(self):
        super().__init__('weed_detector')
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.weed_detected_pub = self.create_publisher(Bool, '/weed_detected', 10)
        
        # State
        self.weed_detected = False
        
        self.get_logger().info('🌱 Weed Detector Node Started!')
        
    def detect_red_weed(self, cv_image):
        """Detect red weeds using HSV color space"""
        try:
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define red color range
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = mask1 + mask2
            
            # Calculate red pixel ratio
            red_pixels = np.sum(red_mask > 0)
            total_pixels = red_mask.shape[0] * red_mask.shape[1]
            red_ratio = red_pixels / total_pixels
            
            if red_ratio > 0.01:  # Detection threshold
                self.get_logger().info(f'🚨 WEED DETECTED! Ratio: {red_ratio:.4f}')
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in weed detection: {str(e)}')
            return False
        
    def navigate_to_weed(self):
        """Simple forward movement when weed detected"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Slow forward
        self.cmd_vel_pub.publish(cmd_vel)
        
    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect weed
            weed_found = self.detect_red_weed(cv_image)
            
            # Publish detection status
            detection_msg = Bool()
            detection_msg.data = weed_found
            self.weed_detected_pub.publish(detection_msg)
            
            if weed_found:
                # Move toward weed
                self.navigate_to_weed()
            else:
                # Stop if no weed detected
                self.stop_robot()
                self.get_logger().info('🔍 Scanning for weeds...', throttle_duration_sec=2.0)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main():
    rclpy.init()
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
