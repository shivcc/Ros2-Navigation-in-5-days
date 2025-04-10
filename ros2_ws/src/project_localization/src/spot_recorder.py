#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from project_localization.srv import SpotSaverMsg
import yaml
import os


class SpotRecorder(Node):
    def __init__(self):
        super().__init__('spot_recorder')
        
        # Dictionary to store spots
        self.spots = {}
        self.current_pose = None
        
        # Create service
        self.service = self.create_service(
            SpotSaverMsg,
            '/save_spot',
            self.save_spot_callback
        )
        
        # Subscribe to AMCL pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        
        self.get_logger().info('Spot recorder service ready')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def save_spot_callback(self, request, response):
        if request.label.lower() == 'end':
            # Save all spots to file
           
            file_path = os.path.join('/home','user', 'ros2_ws','src', 'project_localization', 'config', 'spots.yaml')
            
            try:
                with open(file_path, 'w') as file:
                    yaml.dump(self.spots, file)
                
                response.success = True
                response.message = f"Spots saved to {file_path}"
                self.get_logger().info(response.message)
                
                # Clear spots after saving
                self.spots = {}
                
            except Exception as e:
                response.success = False
                response.message = f"Error saving spots: {str(e)}"
                self.get_logger().error(response.message)
                
        elif self.current_pose is not None:
            # Save current pose with the given label
            self.spots[request.label] = {
                'position': {
                    'x': self.current_pose.position.x,
                    'y': self.current_pose.position.y,
                    'z': self.current_pose.position.z
                },
                'orientation': {
                    'x': self.current_pose.orientation.x,
                    'y': self.current_pose.orientation.y,
                    'z': self.current_pose.orientation.z,
                    'w': self.current_pose.orientation.w
                }
            }
            response.success = True
            response.message = f"Spot '{request.label}' saved (not written to file yet)"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "No pose received yet"
            self.get_logger().warn(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()