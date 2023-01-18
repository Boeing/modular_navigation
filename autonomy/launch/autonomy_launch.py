import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
#from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    navigation_config_launch_arg = DeclareLaunchArgument(
      'navigation_config', default_value=TextSubstitution(text='/home/fran/nav_ros2_ws/src/autonomy/config/navigation.yaml')
    )

    # TODO USE fake_localisation LAUNCH ARG IN IF STATEMENT
    # In case launching fake localisation alongside autonomy
    
    return LaunchDescription([
        navigation_config_launch_arg,
        Node(
            package='autonomy',
            executable='autonomy',
            parameters=[{
                'navigation_config': LaunchConfiguration('navigation_config'),
            }]
        ),
#        Node(
#            package='autonomy',
#            namespace='autonomy',
#            executable='fake_localisation.py'
#        )
    ])