"""
Example to launch a sensor_combined listener node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # micro_ros_agent = ExecuteProcess(
    #     cmd=[[
    #         'micro-ros-agent udp4 --port 8888 -v '
    #     ]],
    #     shell=True
    # )

    drone_control_node = Node(
        package='drone_control',
        executable='drone_control',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        drone_control_node
    ])
