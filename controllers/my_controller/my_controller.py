from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get devices
camera = robot.getDevice('camera')
camera.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Start moving
left_motor.setVelocity(1.0)
right_motor.setVelocity(1.0)

print("Weed Detection - DEBUG MODE")
print("Camera resolution:", camera.getWidth(), "x", camera.getHeight())

# Wait for camera initialization
robot.step(timestep)

def detect_red_simple(img):
    """Simple RGB-based red detection"""
    # Convert BGRA to BGR
    img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    
    # Simple RGB threshold for red
    # Red channel high, Green and Blue channels low
    red_mask = (img_bgr[:, :, 2] > 150) & (img_bgr[:, :, 1] < 100) & (img_bgr[:, :, 0] < 100)
    
    red_ratio = np.sum(red_mask) / red_mask.size
    return red_ratio

def detect_red_hsv(img):
    """HSV-based red detection"""
    img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    
    # Wider red ranges
    lower_red1 = np.array([0, 50, 50])    # More sensitive
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 50, 50])  # More sensitive  
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2
    
    red_ratio = np.sum(red_mask > 0) / red_mask.size
    return red_ratio

frame_count = 0
detection_method = "hsv"  # Try "simple" if hsv doesn't work

while robot.step(timestep) != -1:
    frame_count += 1
    
    img_data = camera.getImage()
    if img_data:
        img = np.frombuffer(img_data, np.uint8).reshape(
            (camera.getHeight(), camera.getWidth(), 4)
        )
        
        if detection_method == "hsv":
            red_ratio = detect_red_hsv(img)
            method_name = "HSV"
        else:
            red_ratio = detect_red_simple(img)
            method_name = "RGB"
        
        print(f"Frame {frame_count:3d} | {method_name} | Red ratio: {red_ratio:.4f}")
        
        # Lower threshold for testing
        if red_ratio > 0.002:  # Very sensitive - just 0.2% red pixels
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("*** WEED DETECTED! STOPPING! ***")
            print(f"*** Detection: {red_ratio:.4f} red pixels found ***")
        else:
            left_motor.setVelocity(1.0)
            right_motor.setVelocity(1.0)
