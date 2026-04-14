# gz_bridge_launch.py
#
# Full simulation launch — starts everything needed to run the weedbot
# pipeline in Gazebo Harmonic without touching real hardware:
#
#   1. Gazebo Harmonic  (with crops + weeds world)
#   2. robot_state_publisher  (URDF → TF tree)
#   3. Spawn weedbot into Gazebo
#   4. ros_gz_bridge  (Gazebo ↔ ROS 2 topics)
#   5. sim_sensor_bridge  (Contacts + Odom → MicroSensorPacket)
#   6. All weedbot_core nodes  (robot_state, safety, behavior, HAL, motor_ctrl)
#
# Usage:
#   ros2 launch weedbot_core gz_bridge_launch.py
#   ros2 launch weedbot_core gz_bridge_launch.py gz_gui:=false   # headless

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    gz_gui = LaunchConfiguration('gz_gui').perform(context)

    # ── Paths ────────────────────────────────────────────────────────────
    sim_pkg = get_package_share_directory('weedbot_simulation_tertill')
    world_file = os.path.join(sim_pkg, 'worlds', 'weedbot.world')
    urdf_file = os.path.join(sim_pkg, 'urdf', 'weedbot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── 1. Gazebo Harmonic ───────────────────────────────────────────────
    gz_cmd = ['gz', 'sim', '-r', world_file]
    if gz_gui in ('false', 'False', '0'):
        gz_cmd.append('--headless-rendering')
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen')

    # ── 2. Robot State Publisher ─────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ── 3. Spawn robot (delayed 2 s to let Gazebo initialise) ────────────
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'weedbot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.10',   # spawn slightly above ground so wheels land cleanly
                ],
                output='screen',
            )
        ],
    )

    # ── 4. ros_gz_bridge ─────────────────────────────────────────────────
    #
    # Format:  /topic@ros_type[gz_type   gz → ros
    #          /topic@ros_type]gz_type   ros → gz
    #          /topic@ros_type@gz_type   bidirectional
    #
    bridge = TimerAction(
        period=3.0,   # after Gazebo + spawn
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    # Clock
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # Drive commands: ROS → Gazebo
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    # Odometry: Gazebo → ROS
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # Contact sensors: Gazebo → ROS
                    '/gazebo/front_contact'
                    '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                    '/gazebo/belly_contact'
                    '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                    # IMU: Gazebo → ROS (if IMU plugin is active)
                    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                ],
                output='screen',
            )
        ],
    )

    # ── 5. Simulation sensor bridge ───────────────────────────────────────
    # Converts Contacts + Odom → MicroSensorPacket
    sim_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='weedbot_core',
                executable='sim_sensor_bridge',
                name='sim_sensor_bridge',
                output='screen',
            )
        ],
    )

    # ── 6. Core nodes (delayed to let sensor bridge come up first) ────────
    core_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(package='weedbot_core', executable='robot_state_node',
                 name='robot_state_node', output='screen'),
            Node(package='weedbot_core', executable='safety_node',
                 name='safety_node', output='screen'),
            Node(package='weedbot_core', executable='behavior_node',
                 name='behavior_node', output='screen'),
            # HAL in mock mode — sim does not need real hardware drivers
            Node(package='weedbot_core', executable='hal_hw_gpio',
                 name='hal_hw_gpio', output='screen',
                 parameters=[{'hw_mode': False, 'hw_backend': 'mock'}]),
            Node(package='weedbot_core', executable='motor_controller',
                 name='motor_controller', output='screen',
                 parameters=[{'hw_mode': False}]),
        ],
    )

    return [gazebo, rsp, spawn, bridge, sim_bridge, core_nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Launch Gazebo with GUI (true) or headless (false)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
