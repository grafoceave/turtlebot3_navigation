# TurtleBot3 Navigation with Cartographer SLAM

Complete autonomous navigation system for TurtleBot3 using ROS 2 Jazzy, Cartographer SLAM for real-time mapping, and Nav2 for path planning and waypoint navigation.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3%20Burger-green)
![SLAM](https://img.shields.io/badge/SLAM-Cartographer-orange)
![Nav2](https://img.shields.io/badge/Navigation-Nav2-red)

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Quick Start Guide](#quick-start-guide)
- [Manual Launch (5 Terminals)](#manual-launch-5-terminals)
- [RViz Configuration](#rviz-configuration)
- [Waypoint Navigation](#waypoint-navigation)
- [Customizing Waypoints](#customizing-waypoints)
- [Troubleshooting](#troubleshooting)
- [Advanced Features](#advanced-features)
- [File Structure](#file-structure)

## 🎯 Overview

This project provides a production-ready autonomous navigation system that combines:

- **Gazebo Simulation** - Realistic physics and sensor simulation
- **Cartographer SLAM** - Real-time 2D mapping and localization
- **Nav2 Stack** - Complete navigation framework with path planning and obstacle avoidance
- **Waypoint Navigation** - Sequential autonomous navigation through multiple goals
- **RViz2** - Real-time visualization of robot state, sensors, and navigation

### System Architecture

Terminal 1: Gazebo
bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Terminal 2: Cartographer SLAM

bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
Terminal 3: RViz

bash
export TURTLEBOT3_MODEL=burger
ros2 run rviz2 rviz2
Set Fixed Frame to map

Add /map display

Add /scan display

Add RobotModel display

Terminal 4: Navigation Stack

bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
Terminal 5: Waypoint Navigation

bash
python3 ~/nav_10_waypoints_final.py
