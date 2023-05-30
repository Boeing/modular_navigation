from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    navigation_config_launch_arg = DeclareLaunchArgument(
      'navigation_config', default_value=TextSubstitution(
        text='/home/fran/nav_ros2_ws/src/autonomy/config/navigation.yaml')
    )

    return LaunchDescription([
        navigation_config_launch_arg,
        Node(
            package='autonomy',
            executable='autonomy',
            parameters=[{
                'navigation_config': LaunchConfiguration('navigation_config'),
            }]
        ),
    ])
