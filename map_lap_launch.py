from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.actions import TimerAction
import os

def generate_launch_description():
    slam_toolbox_path = get_package_share_directory('slam_toolbox')
    robot_path = get_package_share_directory("f1tenth")

    return LaunchDescription([
        # Throttle LIDAR before SLAM starts
        TimerAction(
            period=2.0,  # Wait 2 seconds
            actions=[
                Node(
                    package='topic_tools',
                    executable='throttle',
                    name='lidar_throttle',
                    arguments=['messages', '/scan', '10.0', '/scan_throttled'],
                    output='screen'
                )
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(robot_path,'config/mapper_params_online_async.yaml')}.items(),
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(robot_path, 'config/ekf.yaml')],
        ),

        # LiDAR Node
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

        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # GPIO Node
        Node(
            package='f1tenth',
            executable='jetson_gpio_control',
            name='jetson_gpio_control',
            output='screen'
        ),

        #CmdVel to Odom
        Node(
            package='f1tenth',
            executable='cmd_vel_to_odom.py',
            name='cmdvel_to_odom',
            output='screen',
        )
    ])
