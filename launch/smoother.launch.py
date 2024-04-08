from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop-smoother',
            executable='smoother',
            parameters=[{"linear_acceleration": 0.5,
                         "angular_acceleration":0.5,
                         "input_vel_topic":"cmd_vel",
                         "output_vel_topic":"cmd_vel_smooth"}]
        ),
        
    ])