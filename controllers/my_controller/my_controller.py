from controller import Robot
import numpy as np
import cv2

class RealWorldDeweederController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        
        # Camera
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timestep)
        
        # Wheel motors
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Arm motors
        self.shoulder_motor = self.getDevice('shoulder_motor')
        self.elbow_motor = self.getDevice('elbow_motor')
        
        # Robot state
        self.state = "SEARCHING"
        self.removal_timer = 0
        self.weed_count = 0
        self.state_timer = 0
        self.search_direction = 1  # 1 for right, -1 for left
        self.weed_position = None
        
        print("🌱 REAL-WORLD DEWEEDER ACTIVATED!")
        print("========================================")

    def detect_weed_with_position(self, img):
        """Detect weed and return its position in image coordinates"""
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        # Red detection
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2
        
        red_ratio = np.sum(red_mask > 0) / red_mask.size
        
        # Find weed center if detected
        weed_x = None
        if red_ratio > 0.005:  # Lower threshold for early detection
            moments = cv2.moments(red_mask)
            if moments["m00"] != 0:
                weed_x = int(moments["m10"] / moments["m00"])
        
        return red_ratio, weed_x

    def navigate_toward_weed(self, weed_x):
        """Navigate toward weed based on its position in camera view"""
        img_width = self.camera.getWidth()
        
        if weed_x < img_width * 0.4:  # Weed on left
            self.left_motor.setVelocity(0.5)
            self.right_motor.setVelocity(1.5)
            print("↖️ Turning left toward weed")
        elif weed_x > img_width * 0.6:  # Weed on right
            self.left_motor.setVelocity(1.5)
            self.right_motor.setVelocity(0.5)
            print("↗️ Turning right toward weed")
        else:  # Weed centered
            self.left_motor.setVelocity(1.0)
            self.right_motor.setVelocity(1.0)
            print("⬆️ Weed centered, approaching...")

    def search_pattern(self):
        """Systematic search pattern like real agricultural robots"""
        self.state_timer += 1
        
        # Zig-zag pattern
        if self.state_timer < 100:  # Move forward
            self.left_motor.setVelocity(1.0)
            self.right_motor.setVelocity(1.0)
        elif self.state_timer < 130:  # Turn
            self.left_motor.setVelocity(self.search_direction * 1.5)
            self.right_motor.setVelocity(-self.search_direction * 1.5)
        elif self.state_timer < 230:  # Move sideways
            self.left_motor.setVelocity(1.0)
            self.right_motor.setVelocity(1.0)
        else:  # Reverse direction for next cycle
            self.search_direction *= -1
            self.state_timer = 0

    def move_arm_to_target(self, weed_x):
        """Position arm based on weed location"""
        img_width = self.camera.getWidth()
        
        if weed_x < img_width * 0.4:  # Left side
            self.shoulder_motor.setPosition(0.5)
        elif weed_x > img_width * 0.6:  # Right side
            self.shoulder_motor.setPosition(-0.5)
        else:  # Center
            self.shoulder_motor.setPosition(0.0)
        
        self.elbow_motor.setPosition(0.8)  # Extended for reach

    def perform_removal(self):
        """Realistic removal sequence"""
        self.removal_timer += 1
        
        if self.removal_timer < 20:
            self.elbow_motor.setPosition(1.0)  # Spray forward
            if self.removal_timer == 8:
                print("💦 Applying herbicide...")
            return False
        elif self.removal_timer < 40:
            self.elbow_motor.setPosition(0.6)  # Return
            return False
        else:
            return True

    def reset_arm(self):
        """Return arm to travel position"""
        self.shoulder_motor.setPosition(0.0)
        self.elbow_motor.setPosition(0.3)  # Compact for travel

    def run(self):
        frame_count = 0
        
        while self.step(self.timestep) != -1:
            frame_count += 1
            
            # Get camera image
            img_data = self.camera.getImage()
            if not img_data:
                continue
                
            img = np.frombuffer(img_data, np.uint8).reshape(
                (self.camera.getHeight(), self.camera.getWidth(), 4)
            )
            
            red_ratio, weed_x = self.detect_weed_with_position(img)
            
            # State machine for real-world operation
            if self.state == "SEARCHING":
                self.search_pattern()
                
                if red_ratio > 0.01:  # Weed detected
                    print(f"🔍 Weed spotted! Ratio: {red_ratio:.4f}")
                    self.state = "APPROACHING"
                    self.weed_position = weed_x
                    self.state_timer = 0

            elif self.state == "APPROACHING":
                # Navigate toward weed
                if weed_x is not None:
                    self.navigate_toward_weed(weed_x)
                    self.weed_position = weed_x
                
                # Close enough for removal
                if red_ratio > 0.08:  # Strong detection = close enough
                    print("🎯 In position for removal")
                    self.state = "POSITIONING_ARM"
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state_timer = 0

            elif self.state == "POSITIONING_ARM":
                self.move_arm_to_target(self.weed_position)
                
                if self.state_timer > 40:
                    self.state = "REMOVING"
                    self.removal_timer = 0
                    print("🛠️ Executing removal...")

            elif self.state == "REMOVING":
                removal_complete = self.perform_removal()
                
                if removal_complete:
                    self.weed_count += 1
                    print(f"✅ Weed #{self.weed_count} eliminated!")
                    self.state = "RESETTING"
                    self.state_timer = 0

            elif self.state == "RESETTING":
                self.reset_arm()
                
                if self.state_timer > 50:
                    self.state = "SEARCHING"
                    print("🔍 Resuming search pattern...")

            # Status updates
            if frame_count % 100 == 0:
                print(f"Status: {self.state} | Weeds: {self.weed_count} | Detection: {red_ratio:.4f}")

controller = RealWorldDeweederController()
controller.run()