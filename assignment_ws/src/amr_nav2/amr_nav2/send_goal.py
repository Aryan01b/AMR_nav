#!/usr/bin/env python3
"""
ROS2 Node to send navigation goals to Nav2
Usage: 
  ros2 run amr_nav2 send_goal --ros-args -p x:=2.0 -p y:=1.0
  ros2 run amr_nav2 send_goal --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=1.57
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        
        # Declare parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # Get parameters
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        
        # Create action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Send goal after brief delay to ensure everything is ready
        self.create_timer(1.0, self.send_goal_once)
        
    def send_goal_once(self):
        """Send navigation goal once"""
        self.destroy_timer(self.create_timer(1.0, lambda: None))  # Cancel timer
        self.send_goal(self.x, self.y, self.yaw)
        
    def send_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2"""
        
        # Wait for action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            rclpy.shutdown()
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(
            f'Sending goal: x={x:.2f}m, y={y:.2f}m, yaw={yaw:.2f}rad ({math.degrees(yaw):.1f}°)'
        )
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected!')
            rclpy.shutdown()
            return
            
        self.get_logger().info('✓ Goal accepted! Robot is navigating...')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if distance > 0:
            self.get_logger().info(
                f'📍 Distance remaining: {distance:.2f}m | ⏱ Time: {feedback.navigation_time.sec}s',
                throttle_duration_sec=2.0  # Log every 2 seconds
            )
            
    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        
        if result:
            self.get_logger().info('🎯 Goal reached successfully!')
        else:
            self.get_logger().error('❌ Navigation failed!')
            
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        goal_sender = GoalSenderNode()
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        goal_sender.get_logger().info('⏹ Goal cancelled by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()