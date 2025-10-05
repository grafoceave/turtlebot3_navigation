#!/usr/bin/env python3
"""
Complete TurtleBot3 Navigation and SLAM Launch File
This launch file starts all components needed for autonomous navigation with online SLAM:
1. Gazebo simulation with TurtleBot3 and environment
2. SLAM Toolbox for real-time mapping and localization
3. Nav2 navigation stack for path planning and control
4. RViz2 for visualization

Components are launched with timed delays to ensure proper initialization order.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate the complete launch description with all navigation components.
    Returns a LaunchDescription object that ROS 2 executes.
    """

    # ========================================================================
    # Find Package Paths
    # Locate installed ROS 2 packages to access their launch files and configs
    # ========================================================================
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')  # Simulation package
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')            # Nav2 launch files
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')            # SLAM package
    pkg_this = FindPackageShare('turtlebot_nav2_slam')             # This project's package

    # ========================================================================
    # Launch Arguments
    # Configurable parameters that can be overridden at launch time
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_sim_time tells all nodes to use /clock topic from Gazebo
    # instead of system time. CRITICAL for simulation!

    # ========================================================================
    # Component 1: Gazebo Simulation
    # Launches TurtleBot3 in turtlebot3_world environment with physics
    # ========================================================================
    gazebo = IncludeLaunchDescription(
        # Include the turtlebot3_world launch file from turtlebot3_gazebo package
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )
    # No delay - Gazebo starts immediately as the foundation

    # ========================================================================
    # Component 2: SLAM Toolbox
    # Builds map in real-time and publishes map->odom transform
    # Delayed 5 seconds to allow Gazebo to fully initialize
    # ========================================================================
    slam_params = PathJoinSubstitution([
        pkg_this,
        'config',
        'slam_params.yaml'  # SLAM configuration file
    ])

    slam = TimerAction(
        period=5.0,  # Wait 5 seconds after Gazebo starts
        actions=[
            IncludeLaunchDescription(
                # Use SLAM Toolbox's async (real-time) mapping launch file
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_slam_toolbox,
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                # Pass parameters to SLAM Toolbox
                launch_arguments={
                    'slam_params_file': slam_params,  # Our custom SLAM config
                    'use_sim_time': use_sim_time      # Use Gazebo clock
                }.items()
            )
        ]
    )
    # Delay reason: Gazebo must publish /scan and /odom before SLAM starts

    # ========================================================================
    # Component 3: Nav2 Navigation Stack
    # Starts all Nav2 servers: planner, controller, behavior, bt_navigator
    # Delayed 10 seconds to allow Gazebo and SLAM to initialize
    # ========================================================================
    nav2_params = PathJoinSubstitution([
        pkg_this,
        'config',
        'nav2_params.yaml'  # Nav2 configuration file
    ])

    nav2 = TimerAction(
        period=10.0,  # Wait 10 seconds after launch start
        actions=[
            IncludeLaunchDescription(
                # Use Nav2's standard navigation launch file
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_nav2_bringup,
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                # Pass parameters to Nav2
                launch_arguments={
                    'params_file': nav2_params,    # Our custom Nav2 config
                    'use_sim_time': use_sim_time   # Use Gazebo clock
                }.items()
            )
        ]
    )
    # Delay reason: Nav2 needs map->odom transform from SLAM and sensor data

    # ========================================================================
    # Component 4: RViz2 Visualization
    # Displays robot, sensors, map, costmaps, and navigation paths
    # Delayed 12 seconds to allow all other components to start
    # ========================================================================
    rviz_config = PathJoinSubstitution([
        pkg_nav2_bringup,
        'rviz',
        'nav2_default_view.rviz'  # Pre-configured RViz layout for Nav2
    ])

    rviz = TimerAction(
        period=12.0,  # Wait 12 seconds after launch start
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],  # Load config file
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    # Delay reason: RViz should start after topics are being published

    # ========================================================================
    # Return Complete Launch Description
    # All components bundled together with proper initialization order
    # ========================================================================
    return LaunchDescription([
        # Declare launch arguments that users can override
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch all components in order
        gazebo,  # T+0s: Simulation environment
        slam,    # T+5s: SLAM mapping
        nav2,    # T+10s: Navigation stack
        rviz     # T+12s: Visualization
    ])
