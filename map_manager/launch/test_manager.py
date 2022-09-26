from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_manager',
            executable='map_manager_node.py',
            name='map_manager',
            namespace='map_manager',
            output='screen',
            remappings=[
                ('/map_manager/pbstream', '/mapper/map_data'),
            ]
        )
    ])
