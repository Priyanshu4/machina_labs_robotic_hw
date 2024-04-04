"""
Launches the sensor simulator located at scripts/sensor.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

sensor_simulator_py_file = os.path.join(
    get_package_share_directory('robotic_hw'),
    'scripts',
    'sensor.py'
)

def generate_launch_description():
    return LaunchDescription([

        # Run the sensor simulator
        ExecuteProcess(
            cmd=['python3', sensor_simulator_py_file],
            output='screen'
        )
    ])