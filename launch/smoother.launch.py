from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,\
    PythonExpression

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plot',
            default_value='false',
            description="controls wether the plotting node is launched.\
                  Values: ['true','false']"),
        Node(
            package='teleop-smoother',
            executable='smoother',
            parameters=[{"linear_acceleration": 0.5,
                         "angular_acceleration":0.5,
                         "input_vel_topic":"cmd_vel",
                         "output_vel_topic":"cmd_vel_smooth",
                         "loop_frequency":50.0}]
        ),
        Node(
            package='teleop-smoother',
            executable='plotter',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('plot'), '\'', '==\'true\''])),
            parameters=[{"raw_vel_topic":"cmd_vel",
                         "smooth_vel_topic":"cmd_vel_smooth",
                         "loop_frequency":50.0
                         }]
        )
        
    ])