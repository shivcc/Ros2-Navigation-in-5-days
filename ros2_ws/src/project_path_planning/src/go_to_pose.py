#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import os


class MoveToSpot(Node):
    def __init__(self):
        super().__init__('move_to_spot')
        
        # Declare parameter
        self.declare_parameter('spot_name')
        
        # Load spots from parameter file
        file_path = os.path.join('/home', 'user', 'ros2_ws', 'src', 
                                   'project_path_planning', 'spots', 'spots.yaml')
        
        with open(file_path, 'r') as file:
            params = yaml.safe_load(file)
            self.spots = params['move_to_spot']['ros__parameters']['spots']
        
        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('Move to spot action client ready')

    def send_goal(self):
        spot_name = self.get_parameter('spot_name').get_parameter_value().string_value
        
        if spot_name not in self.spots:
            self.get_logger().error(f'Spot "{spot_name}" not found in spots list')
            return
            
        spot = self.spots[spot_name]
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = spot['position']['x']
        goal_pose.pose.position.y = spot['position']['y']
        goal_pose.pose.position.z = spot['position']['z']
        
        # Set orientation
        goal_pose.pose.orientation.x = spot['orientation']['x']
        goal_pose.pose.orientation.y = spot['orientation']['y']
        goal_pose.pose.orientation.z = spot['orientation']['z']
        goal_pose.pose.orientation.w = spot['orientation']['w']
        
        # Create action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        self.get_logger().info(f'Sending robot to {spot_name}...')
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} meters'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result.result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToSpot()
    
    try:
        action_client.send_goal()
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Navigation canceled')
    finally:
        action_client.destroy_node()

if __name__ == '__main__':
    main()