"""
This launch file launches two_sensors.launch .py and simulator.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

two_sensors_launch_file = os.path.join(
    get_package_share_directory('robotic_hw'),
    'launch',
    'two_sensors.launch.py'
)

simulator_launch_file = os.path.join(
    get_package_share_directory('robotic_hw'),
    'launch',
    'simulator.launch.py'
)


def generate_launch_description():
    return LaunchDescription([

       # Launch the sensor simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([simulator_launch_file])
        ),

        # Launch the nodes from two_sensors.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([two_sensors_launch_file])
        )
    ])