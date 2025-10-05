#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    pkg_this = FindPackageShare('turtlebot_nav2_slam')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py'])
        ])
    )
    
    # SLAM Toolbox
    slam_params = PathJoinSubstitution([pkg_this, 'config', 'slam_params.yaml'])
    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
                ]),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # Nav2
    nav2_params = PathJoinSubstitution([pkg_this, 'config', 'nav2_params.yaml'])
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
                ]),
                launch_arguments={
                    'params_file': nav2_params,
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'])
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        slam,
        nav2,
        rviz
    ])
