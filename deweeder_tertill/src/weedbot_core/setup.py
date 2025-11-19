from setuptools import setup
from glob import glob
import os

package_name = 'weedbot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # install launch files so ros2 launch can find them
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # (optional) install any node-related config or other assets here
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='weedbot core nodes and launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_state_node = weedbot_core.robot_state_node:main',
            'behavior_node = weedbot_core.behavior_node:main',
            'safety_node = weedbot_core.safety_node:main',
        ],
    },
)
