#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('turtlebot_nav2_slam')
    params_file = PathJoinSubstitution([pkg, 'config', 'nav2_twist.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Controller - publishes only Twist
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'enable_stamped_cmd_vel': False  # Force Twist only
            }]
        ),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        # Lifecycle manager - only these 4 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ])
