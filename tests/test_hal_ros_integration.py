"""Optional real ROS/DDS integration test. Never constructs a physical driver.

Run after colcon build and sourcing ROS + workspace. A skip is NOT a pass.
"""
import importlib.util
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deweeder_tertill/src/weedbot_core'))
HAS_ROS = all(importlib.util.find_spec(name) is not None for name in
              ('rclpy', 'weedbot_msgs', 'std_srvs', 'nav_msgs'))


@unittest.skipUnless(HAS_ROS, 'ROS runtime and built weedbot_msgs required')
class HalRosIntegration(unittest.TestCase):
    def test_safety_accepts_generated_imu_covariances_and_rejects_invalid(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Imu
        from weedbot_core.safety_node import SafetyNode
        rclpy.init(args=[])
        node = SafetyNode()
        try:
            node._state_cb(Twist())
            imu = Imu()
            imu.linear_acceleration.z = 9.81
            node._imu_cb(imu)
            self.assertEqual(node._monitor.evaluate().state, 0)
            imu.linear_acceleration_covariance[0] = -1.0
            node._imu_cb(imu)
            self.assertEqual(node._monitor.evaluate().state, 2)
            imu.linear_acceleration_covariance[0] = float('nan')
            node._imu_cb(imu)
            self.assertEqual(node._monitor.evaluate().state, 2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_dds_emergency_stop_service_rearm_and_timer_expiry(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float32
        from std_srvs.srv import Trigger
        from nav_msgs.msg import Odometry
        from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd
        from weedbot_core.hal_hw_gpio import HalHW
        rclpy.init(args=[])
        hal = HalHW()
        source = Node('hal_safety_test_source')
        executor = SingleThreadedExecutor()
        executor.add_node(hal)
        executor.add_node(source)
        pubs = {key: source.create_publisher(kind, topic, 1) for key, kind, topic in (
            ('heartbeat', Float32, '/heartbeat'), ('safety', SafetyCmd, '/safety_cmd'),
            ('motion', MotionCmd, '/motion_cmd'), ('tool', ToolCmd, '/tool_cmd'),
            ('odom', Odometry, '/odom'), ('estop', Bool, '/emergency_stop'))}
        arm = source.create_client(Trigger, '/hal/arm')
        moving = False
        def publish():
            pubs['heartbeat'].publish(Float32(data=1.0))
            pubs['safety'].publish(SafetyCmd(state=0))
            odom = Odometry()
            odom.twist.twist.linear.x = 0.1 if moving else 0.0
            pubs['odom'].publish(odom)
            pubs['motion'].publish(MotionCmd(linear_x=0.2 if moving else 0.0, angular_z=0.0))
            pubs['tool'].publish(ToolCmd())
        timer = source.create_timer(0.05, publish)
        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
        def request_arm():
            self.assertTrue(arm.wait_for_service(timeout_sec=2.0))
            future = arm.call_async(Trigger.Request())
            end = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
        try:
            spin(1.0)
            self.assertFalse(hal.gate.armed)
            request_arm()
            moving = True
            spin(0.2)
            self.assertTrue(any(hal.driver._last_pwm))
            pubs['estop'].publish(Bool(data=True))
            spin(0.1)
            self.assertFalse(hal.gate.armed)
            self.assertEqual(hal.driver._last_pwm, [0.0] * 4)
            moving = False
            pubs['estop'].publish(Bool(data=False))
            spin(0.2)
            self.assertFalse(hal.gate.armed)
            request_arm()
            moving = True
            spin(0.2)
            self.assertTrue(any(hal.driver._last_pwm))
            timer.cancel()
            spin(0.7)
            self.assertFalse(hal.gate.armed)
            self.assertEqual(hal.driver._last_pwm, [0.0] * 4)
            self.assertEqual(hal.driver._last_tool, (False, 0, False))
        finally:
            executor.remove_node(hal)
            executor.remove_node(source)
            hal.destroy_node()
            source.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
