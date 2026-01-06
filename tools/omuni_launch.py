from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),

        Node(
            package='omuni_robot',
            executable='omuni_3rin_node',
            output='screen'
        ),

    ])
