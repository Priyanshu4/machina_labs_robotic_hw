"""
This launch file is used to launch the two load cell data servers and the load cell data publisher node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        
        # Launch the first server node for the first sensor
        # The number of samples and request_frequency parametes are chosen based on sensor sample rate and delay
        # I choose them such that more time is spent on sampling than the delay. 
        # However, we also do not want the number of samples to be too large as this adds latency.
        # The first sensor has a sample rate of 2000 Hz and a delay of 1 ms. Each sensor in the simulator also has additional random latency up to 1ms.
        # By taking 10 samples at a time, 5 ms are spent on sampling and up to 2 ms on delay. Total time is 7 ms which allows a rate greater than 100 hz.
        Node(
            package='robotic_hw',
            executable='load_cell_data_server',
            name='load_cell_data_server_1',
            output='screen',
            namespace='sensor_1',
            parameters=[{'server_address': '127.0.0.3'},
                        {'server_port': 10000},
                        {'number_of_samples': 10}]),
        
        # Launch the second  server node for the second sensor
        # The second sensor has a sample rate of 4000 Hz and a delay of 3 ms
        # Because the delay is longer and the sample rate is faster, the number of samples is increased.
        # By taking 24 samples at a time, 6 ms are spent on sampling and up to 4 ms on delay. Total time is 10 ms which allows 100 hz rate.
        Node(
            package='robotic_hw',
            executable='load_cell_data_server',
            name='load_cell_data_server_2',
            output='screen',
            namespace='sensor_2',
            parameters=[{'server_address': '127.0.0.1'},
                        {'server_port': 10000},
                        {'number_of_samples': 24}]),

        # Launch the load cell data publisher node
        Node(
            package='robotic_hw',
            executable='load_cell_data_publisher',
            name='load_cell_data_publisher',
            output='screen',
            parameters=[{'service_names': ['sensor_1/get_load_cell_data', 'sensor_2/get_load_cell_data']},
                        {'publish_frequency': 500.0},
                        {'service_call_frequency': 100.0}])

    ])
