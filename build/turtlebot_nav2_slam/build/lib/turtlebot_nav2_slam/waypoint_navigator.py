#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info('Waypoint Navigator Ready')
    
    def send_waypoints(self, waypoints):
        self.get_logger().info(f'Waiting for action server...')
        self.action_client.wait_for_server()
        
        goal_msg = NavigateThroughPoses.Goal()
        
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)
        
        self.get_logger().info(f'Sending {len(waypoints)} waypoints')
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('✓ Navigation complete!')

def main():
    rclpy.init()
    navigator = WaypointNavigator()
    
    # Define 10 waypoints
    waypoints = [
        (1.0, 0.5), (2.0, 1.0), (3.0, 0.5),
        (3.5, 2.0), (2.5, 3.0), (1.0, 3.5),
        (-0.5, 3.0), (-1.0, 2.0), (-0.5, 1.0), (0.0, 0.0)
    ]
    
    time.sleep(2.0)  # Wait for Nav2 to initialize
    navigator.send_waypoints(waypoints)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
