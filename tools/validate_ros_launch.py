"""Opt-in installed-package ROS acceptance. Mock only; localhost domain required."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from weedbot_msgs.msg import MicroSensorPacket, MotionCmd, ToolCmd, SafetyCmd
from weedbot_core.hal_hw_gpio import HalHW

OUT=Path(os.environ['ROS_ACCEPTANCE_OUT'])
OUT.mkdir(parents=True,exist_ok=True)
EVENTS=[]

def record(name, **data):
    EVENTS.append(dict(case=name,monotonic=time.monotonic(),**data))
    (OUT/'observations.json').write_text(json.dumps(EVENTS,indent=2)+'\n')

class RosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.source=Node('acceptance_source')
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.source)

    def spin(self, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: self.executor.spin_once(timeout_sec=.01)

    def call(self, name, expected=True):
        client=self.source.create_client(Trigger,name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5),name)
            future=client.call_async(Trigger.Request())
            end=time.monotonic()+3
            while not future.done() and time.monotonic()<end: self.executor.spin_once(timeout_sec=.01)
            self.assertTrue(future.done(),name)
            result=future.result()
            self.assertEqual(result.success,expected,(name,result.message))
            return result.message
        finally: self.source.destroy_client(client)

    def tearDown(self):
        self.executor.remove_node(self.source)
        self.source.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

class InstalledLaunch(RosTest):
    def test_whole_launch(self):
        log=(OUT/'launch.txt').open('w')
        proc=subprocess.Popen(['ros2','launch','weedbot_core','robot_core.launch.py',
            'hw_mode:=False','hw_backend:=mock','use_behavior:=False'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        heartbeat=None
        state={'ready':None,'safety':None,'diag':None}
        subscriptions=[]
        for key,typ,topic in [('ready',Bool,'/hal/ready'),('safety',SafetyCmd,'/safety_cmd'),('diag',String,'/hal/diag')]:
            def receive(msg,key=key):
                value=msg.state if key=='safety' else msg.data
                if state[key]!=value: record('launch_topic',topic=key,value=value)
                state[key]=value
            subscriptions.append(self.source.create_subscription(typ,topic,receive,10))
        enabled={'sensor':False,'heartbeat':True,'motion':True,'tool':True}
        motion={'moving':False,'tools':False,'unsafe':False,'ticks':0}
        pubs={key:self.source.create_publisher(typ,topic,10) for key,typ,topic in [
            ('sensor',MicroSensorPacket,'/micro/sensor_packet'),('heartbeat',Float32,'/heartbeat'),
            ('motion',MotionCmd,'/motion_cmd'),('tool',ToolCmd,'/tool_cmd'),('estop',Bool,'/emergency_stop')]}
        def feed():
            if enabled['sensor']:
                motion['ticks']+=50 if motion['moving'] else 0
                msg=MicroSensorPacket(wheel_encoder=[motion['ticks']]*4,imu_linear=[0.,0.,-9.81 if motion['unsafe'] else 9.81],tick_time=.05)
                pubs['sensor'].publish(msg)
            if enabled['heartbeat']: pubs['heartbeat'].publish(Float32(data=1.))
            if enabled['motion']: pubs['motion'].publish(MotionCmd(linear_x=.2 if motion['moving'] else 0.))
            if enabled['tool']: pubs['tool'].publish(ToolCmd(front_trimmer_on=motion['tools'],belly_trimmer_mode=2 if motion['tools'] else 0,agitator_pulse=motion['tools']))
        timer=self.source.create_timer(.05,feed)
        def prepare():
            motion.update(moving=False,tools=False,unsafe=False)
            enabled.update(sensor=True,heartbeat=True,motion=True,tool=True)
            self.spin(.8); self.call('/safety/reset'); self.spin(.2); self.call('/hal/arm')
            motion['moving']=True; self.spin(.2)
            motion['tools']=True; self.spin(.3)
            self.assertTrue(state['ready'],state)
        try:
            expected={'robot_state_node','safety_node','hal_hw_gpio','motor_controller'}
            deadline=time.monotonic()+12
            while time.monotonic()<deadline and not (expected<=set(self.source.get_node_names()) and state['ready'] is not None and state['safety'] is not None):
                self.spin(.1)
            self.assertIsNone(proc.poll(),'launch exited')
            names=set(self.source.get_node_names())
            self.assertTrue({'robot_state_node','safety_node','hal_hw_gpio','motor_controller'}<=names,names)
            self.assertNotIn('behavior_node',names)
            self.assertFalse(state['ready'])
            self.assertIn(state['safety'],(2,3))
            self.call('/hal/arm',False)
            record('launch_missing_sensors',passed=True,nodes=sorted(names),state=dict(state))
            prepare()
            pubs['estop'].publish(Bool(data=True)); self.spin(.3)
            self.assertFalse(state['ready'])
            pubs['estop'].publish(Bool(data=False)); self.spin(.3)
            self.assertFalse(state['ready'])
            record('launch_estop_no_auto_restart',passed=True)
            prepare()
            motion['unsafe']=True; self.spin(.3)
            self.assertEqual(state['safety'],3)
            self.assertFalse(state['ready'])
            self.call('/safety/reset',False)
            record('launch_unsafe_reset_rejected',passed=True)
            for key in ('sensor','heartbeat','motion','tool'):
                prepare(); enabled[key]=False
                start=time.monotonic(); self.spin(.85)
                self.assertFalse(state['ready'],(key,state))
                record('launch_source_loss',source=key,passed=True,observation_window_s=time.monotonic()-start,diag=state['diag'])
            prepare()
            heartbeat=subprocess.Popen([sys.executable,__file__,'--heartbeat'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
            self.spin(1.5); enabled['heartbeat']=False; self.spin(.3)
            self.assertIsNone(heartbeat.poll()); self.assertTrue(state['ready'])
            heartbeat.terminate(); heartbeat.wait(timeout=5)
            self.spin(.85); self.assertFalse(state['ready'])
            record('launch_publisher_process_termination',passed=True,diag=state['diag'])
        finally:
            timer.cancel()
            if heartbeat is not None and heartbeat.poll() is None:
                heartbeat.terminate(); heartbeat.wait(timeout=5)
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try: proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL); proc.wait(); self.fail('Launch did not shut down within 10 s')
            log.close()
            record('launch_shutdown',returncode=proc.returncode)
        text=(OUT/'launch.txt').read_text()
        self.assertNotIn('Traceback',text)
        self.assertNotIn('process has died',text)
        self.assertEqual(proc.returncode,0,text[-2000:])

class InstalledHalOutputs(RosTest):
    def test_independent_timeouts_paused_clock_and_shutdown(self):
        hal=HalHW(); self.executor.add_node(hal)
        enabled={k:True for k in ('heartbeat','safety','feedback','motion','tool')}
        active={'motion':False,'tool':False}
        pubs={k:self.source.create_publisher(typ,topic,10) for k,typ,topic in [
            ('heartbeat',Float32,'/heartbeat'),('safety',SafetyCmd,'/safety_cmd'),
            ('feedback',Odometry,'/odom'),('motion',MotionCmd,'/motion_cmd'),('tool',ToolCmd,'/tool_cmd')]}
        def feed():
            odom=Odometry(); odom.twist.twist.linear.x=.1 if active['motion'] else 0.
            msgs=dict(heartbeat=Float32(data=1.),safety=SafetyCmd(state=0),feedback=odom,
                motion=MotionCmd(linear_x=.2 if active['motion'] else 0.),
                tool=ToolCmd(front_trimmer_on=active['tool'],belly_trimmer_mode=2 if active['tool'] else 0,agitator_pulse=active['tool']))
            for key,msg in msgs.items():
                if enabled[key]: pubs[key].publish(msg)
        timer=self.source.create_timer(.05,feed)
        def prepare():
            active.update(motion=False,tool=False); enabled.update({k:True for k in enabled})
            self.spin(.7); self.call('/hal/arm')
            active['motion']=True; self.spin(.15)
            active['tool']=True; self.spin(.15)
            self.assertTrue(any(hal.driver._last_pwm))
            self.assertEqual(hal.driver._last_tool,(True,2,True))
        def stopped(case):
            self.assertFalse(hal.gate.armed)
            self.assertEqual(hal.driver._last_pwm,[0.]*4)
            self.assertEqual(hal.driver._last_tool,(False,0,False))
            record(case,passed=True,pwm=hal.driver._last_pwm,tool=hal.driver._last_tool,reason=hal.gate.reason)
        estop=self.source.create_publisher(Bool,'/emergency_stop',10)
        destroyed=False
        try:
            prepare(); estop.publish(Bool(data=True)); self.spin(.2)
            stopped('hal_estop_active_tools')
            estop.publish(Bool(data=False)); self.spin(.2)
            stopped('hal_estop_release_no_restart')
            for key in enabled:
                prepare(); enabled[key]=False; self.spin(.7)
                stopped('hal_independent_'+key+'_loss')
                self.assertIn(key,hal.gate.reason)
            prepare()
            result=hal.set_parameters([Parameter('use_sim_time',value=True)])
            self.assertTrue(result[0].successful)
            self.assertEqual(hal.get_clock().now().nanoseconds,0)
            enabled['heartbeat']=False; self.spin(.7)
            stopped('hal_watchdog_with_frozen_ros_clock')
            self.assertEqual(hal.get_clock().now().nanoseconds,0)
            prepare(); timer.cancel()
            self.executor.remove_node(hal); hal.destroy_node(); destroyed=True
            stopped('hal_shutdown_active_tools')
            self.assertFalse(hal.driver.is_healthy())
        finally:
            timer.cancel()
            if not destroyed:
                self.executor.remove_node(hal); hal.destroy_node()

class InstalledSafetyInputs(RosTest):
    def test_independent_sensor_expiry_with_frozen_ros_clock(self):
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Imu
        from weedbot_core.safety_node import SafetyNode
        node=SafetyNode()
        self.executor.add_node(node)
        node.set_parameters([Parameter('use_sim_time',value=True)])
        pubs={'contacts':self.source.create_publisher(Twist,'/robot_state',10),
              'imu':self.source.create_publisher(Imu,'/imu',10)}
        enabled={'contacts':True,'imu':True}
        def feed():
            if enabled['contacts']: pubs['contacts'].publish(Twist())
            if enabled['imu']:
                imu=Imu(); imu.linear_acceleration.z=9.81
                pubs['imu'].publish(imu)
        timer=self.source.create_timer(.05,feed)
        try:
            for key in enabled:
                enabled.update(contacts=True,imu=True)
                self.spin(.8)
                self.assertEqual(node._monitor.evaluate().state,0)
                enabled[key]=False; self.spin(.8)
                self.assertEqual(node._monitor.evaluate().state,2)
                self.assertEqual(node.get_clock().now().nanoseconds,0)
                self.call('/safety/reset',False)
                record('safety_frozen_clock_'+key+'_expiry',passed=True,reason=node._monitor.evaluate().reason)
        finally:
            timer.cancel(); self.executor.remove_node(node); node.destroy_node()

def heartbeat_main():
    rclpy.init(); node=Node('acceptance_external_heartbeat')
    pub=node.create_publisher(Float32,'/heartbeat',10)
    timer=node.create_timer(.05,lambda:pub.publish(Float32(data=1.)))
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__':
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Explicit isolated localhost ROS domain required')
    if '--heartbeat' in sys.argv: heartbeat_main()
    else: unittest.main(verbosity=2)
