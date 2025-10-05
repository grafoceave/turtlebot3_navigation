#!/usr/bin/env python3
"""
TurtleBot3 Waypoint Navigator using NavigateThroughPoses Action
This script sends multiple waypoints simultaneously to Nav2 for sequential navigation.
Uses NavigateThroughPoses which is more efficient than sending individual goals.
"""

import rclpy  # ROS 2 Python client library for core functionality
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from rclpy.action import ActionClient  # Client interface for ROS 2 actions
from nav2_msgs.action import NavigateThroughPoses  # Action type for multi-waypoint navigation
from geometry_msgs.msg import PoseStamped  # Message type for poses with timestamps and frame info
import time  # Python time module for delays


class WaypointNavigator(Node):
    """
    ROS 2 Node for batch waypoint navigation through Nav2.
    NavigateThroughPoses allows sending all waypoints at once rather than sequentially.
    This enables Nav2 to optimize the entire path and handle failures more gracefully.
    """

    def __init__(self):
        """
        Initialize the waypoint navigator node.
        Sets up the action client connection to Nav2's waypoint follower.
        """
        # Initialize parent Node class with unique node name for ROS 2 graph
        super().__init__('waypoint_navigator')

        # Create action client for NavigateThroughPoses
        # This action accepts multiple poses and navigates through them sequentially
        # The action server is provided by Nav2's bt_navigator node
        self.action_client = ActionClient(
            self,
            NavigateThroughPoses,  # Action type
            'navigate_through_poses'  # Action name/topic
        )

        # Log successful initialization for debugging
        self.get_logger().info('Waypoint Navigator Ready')

    def send_waypoints(self, waypoints):
        """
        Send a list of waypoints to Nav2 for sequential navigation.

        Args:
            waypoints (list): List of (x, y) tuples representing waypoint coordinates in map frame

        The function waits for the action server, constructs the goal message,
        sends it, and monitors completion.
        """
        # Wait for Nav2's action server to become available
        # This blocks until the server is ready to accept goals
        self.get_logger().info(f'Waiting for action server...')
        self.action_client.wait_for_server()

        # Create empty goal message for NavigateThroughPoses action
        goal_msg = NavigateThroughPoses.Goal()

        # Convert each (x, y) waypoint tuple into a PoseStamped message
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()  # Create pose with header and position/orientation

            # Set header information for coordinate frame reference
            pose.header.frame_id = 'map'  # Waypoints are in the map frame (created by SLAM)
            pose.header.stamp = self.get_clock().now().to_msg()  # Current timestamp

            # Set position coordinates (z=0 for ground navigation)
            pose.pose.position.x = x  # X coordinate in meters
            pose.pose.position.y = y  # Y coordinate in meters
            pose.pose.position.z = 0.0  # Height (always 0 for differential drive robots)

            # Set orientation using quaternion representation
            # w=1.0 with x=y=z=0.0 means no rotation (facing forward)
            # This lets the robot choose its orientation at each waypoint
            pose.pose.orientation.w = 1.0

            # Add this pose to the goal's list of waypoints
            goal_msg.poses.append(pose)

        # Log how many waypoints are being sent
        self.get_logger().info(f'Sending {len(waypoints)} waypoints')

        # Send goal asynchronously and get a future object
        # Async allows us to monitor progress while the goal executes
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        # Block until Nav2 accepts or rejects the goal
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Get the goal handle which tracks this specific navigation task
        goal_handle = send_goal_future.result()

        # Check if goal was accepted by Nav2
        if not goal_handle.accepted:
            # Goal rejected - typically means waypoints are invalid (off map, in obstacles)
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted, navigating...')

        # Wait for navigation to complete (or fail)
        # Nav2 will navigate through all waypoints sequentially
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Log completion (check result.status for detailed success/failure info)
        self.get_logger().info('✓ Navigation complete!')


def main():
    """
    Main entry point for the waypoint navigation script.
    Initializes ROS 2, creates navigator, sends waypoints, and cleans up.
    """
    # Initialize ROS 2 Python client library
    # Must be called before any ROS 2 functionality
    rclpy.init()

    # Create waypoint navigator node instance
    navigator = WaypointNavigator()

    # Define 10 waypoints as (x, y) coordinate tuples in meters
    # These coordinates are relative to the map frame origin (where SLAM started)
    # Coordinates should be verified in RViz to ensure they're in free space
    waypoints = [
        (1.0, 0.5),    # Waypoint 1: 1m forward, 0.5m right
        (2.0, 1.0),    # Waypoint 2: Continue forward-right
        (3.0, 0.5),    # Waypoint 3: Move right
        (3.5, 2.0),    # Waypoint 4: Move up significantly
        (2.5, 3.0),    # Waypoint 5: Continue upward, move left
        (1.0, 3.5),    # Waypoint 6: Further left and up
        (-0.5, 3.0),   # Waypoint 7: Move into negative x (behind start)
        (-1.0, 2.0),   # Waypoint 8: Continue backward-down
        (-0.5, 1.0),   # Waypoint 9: Return toward origin
        (0.0, 0.0)     # Waypoint 10: Return to start position (home)
    ]

    # Wait briefly for Nav2 to fully initialize
    # This prevents sending goals before costmaps and planners are ready
    time.sleep(2.0)

    # Send all waypoints to Nav2 and wait for completion
    navigator.send_waypoints(waypoints)

    # Shutdown ROS 2 cleanly
    rclpy.shutdown()


# Standard Python idiom - only run main() when script is executed directly
if __name__ == '__main__':
    main()
