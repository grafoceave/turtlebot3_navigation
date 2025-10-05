#!/usr/bin/env python3
"""
Nav2-Only Launch File
Launches just the Nav2 navigation stack without Gazebo, SLAM, or RViz.
Useful when these components are already running in separate terminals,
or for testing Nav2 configuration independently.

This manually starts individual Nav2 servers instead of using nav2_bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description with Nav2 navigation servers only.
    Each Nav2 component is launched as a separate node for fine-grained control.
    """

    # Find this package to locate configuration files
    pkg = FindPackageShare('turtlebot_nav2_slam')

    # Path to complete Nav2 parameters file
    # This file contains all Nav2 server configurations
    params_file = PathJoinSubstitution([
        pkg,
        'config',
        'nav2_complete.yaml'
    ])

    # Launch configuration variables that can be set at runtime
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    # autostart=true means lifecycle nodes transition to active immediately

    return LaunchDescription([
        # ====================================================================
        # Declare Launch Arguments
        # Users can override these when launching: ros2 launch ... use_sim_time:=false
        # ====================================================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from /clock topic'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically transition lifecycle nodes to active state'
        ),

        # ====================================================================
        # Controller Server
        # Executes local trajectories to follow the global plan
        # Generates /cmd_vel commands based on costmaps and critics
        # ====================================================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',  # Print logs to terminal
            parameters=[params_file, {'use_sim_time': use_sim_time}]
            # Parameters come from YAML file + runtime overrides
        ),

        # ====================================================================
        # Planner Server
        # Computes global paths from start to goal
        # Uses NavFn (Dijkstra/A*) algorithm on the global costmap
        # ====================================================================
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # ====================================================================
        # Behavior Server
        # Provides recovery behaviors: spin, backup, wait, etc.
        # Called by BT Navigator when primary navigation fails
        # ====================================================================
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # ====================================================================
        # BT Navigator
        # Orchestrates navigation using behavior trees
        # Coordinates planner, controller, and recovery behaviors
        # Provides the /navigate_to_pose and /navigate_through_poses actions
        # ====================================================================
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # ====================================================================
        # Lifecycle Manager
        # Manages lifecycle state transitions for all Nav2 nodes
        # Automatically configures and activates nodes if autostart=true
        # ====================================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                # List of lifecycle nodes to manage
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        )
        # Lifecycle manager ensures all servers start in the correct order
    ])
