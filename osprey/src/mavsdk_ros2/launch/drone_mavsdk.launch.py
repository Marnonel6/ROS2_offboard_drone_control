"""
Launch file
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        # Start the mavsdk_server that connects over serial (UART) to the Pixhawk FC
        ExecuteProcess(cmd=[['echo \'osprey\' | sudo -S \
                            ~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server \
                            serial:///dev/serial0:921600'
                            ]],
                       shell=True
                    ),
        # Start the drone_data node
        Node(package='mavsdk_ros2',
             executable='drone_data',
             output='screen',
            ),
    ])
