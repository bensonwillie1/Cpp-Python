from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.actions import TimerAction
import os

def generate_launch_description():

    waypoint_file = LaunchConfiguration('waypoint_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='/home/f1tenth/F1tenthcpp/f1tenth_cpp_ws/maps/middleline.csv',
            description='Path to waypoint .csv file'
        ),

        Node(
            package='f1tenth',
            executable='waypointfollower.py',
            name='waypoint_follower',
            output='screen',
            parameters=[{'waypoint_file': waypoint_file}]
        ),

        Node(
            package='f1tenth',
            executable='cmd_vel_to_odom.py',
            name='cmd_vel_to_odom',
            output='screen'
        ),


        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['map_server', 'slam_toolbox']
                    }]
                )
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'localization_launch.py')),
            launch_arguments={'slam_params_file': '/home/f1tenth/F1tenthcpp/f1tenth_cpp_ws/src/f1tenth/config/slam_localization.yaml'}.items(),
        ),


        Node(
            package='f1tenth',
            executable='jetson_gpio_control',
            name='jetson_gpio_control',
            output='screen'
        ),

        Node(
            package='f1tenth',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen',
        ),

        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            parameters=[{
                'ip_address': '192.168.0.10',
                'ip_port': 10940,
                'frame_id': 'laser',
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        # ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/f1tenth/F1tenthcpp/f1tenth_cpp_ws/maps/test.yaml'
            }]
        ),
    ])
