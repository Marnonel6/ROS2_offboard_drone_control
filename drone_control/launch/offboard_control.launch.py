"""
Launch file for off-board drone control on a companion computer over DDS
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        # TODO Start the MicroXRCEAgent DDS if on the RPi that connects over serial (UART) to the Pixhawk FC
#         ExecuteProcess(
#                         cmd=[['echo \'osprey\' | sudo -S \
#                             MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600'
#                             ]],
#                         shell=True
#                       ),
        # With Gazebo SITL start MicroXRCEAgent DDS with udp connection
        ExecuteProcess(
                       cmd=[[
                             'MicroXRCEAgent udp4 --port 8888 -v'
                            ]],
                       shell=True
                      ),
        Node(
             package='drone_control',
             executable='drone_control',
             output='screen',
             shell=True,
            ),
        Node(
             package='drone_control',
             executable='path_planning',
             output='screen',
             shell=True,
            ),
    ])