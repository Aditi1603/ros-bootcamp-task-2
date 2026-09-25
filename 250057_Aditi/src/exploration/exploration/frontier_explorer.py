import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Subscribe to local map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
            
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_goal = None
        self.map_data = None
        self.map_info = None
        self.exploring = False

        self.timer = self.create_timer(5.0, self.explore)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def explore(self):
        if self.map_data is None or self.exploring:
            return

        # Find frontiers
        # Free space = 0, Unknown = -1, Obstacle = 100
        free_space = (self.map_data == 0).astype(np.uint8) * 255
        unknown_space = (self.map_data == -1).astype(np.uint8) * 255

        # Dilate free space slightly to find boundaries
        kernel = np.ones((3,3), np.uint8)
        dilated_free = cv2.dilate(free_space, kernel, iterations=1)
        
        # Frontier is where dilated free space overlaps with unknown space
        frontier_map = cv2.bitwise_and(dilated_free, unknown_space)

        contours, _ = cv2.findContours(frontier_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_frontiers = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 5.0: # Filter small frontiers
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    valid_frontiers.append((cx, cy))

        if not valid_frontiers:
            self.get_logger().info('No more frontiers found. Exploration complete.')
            return

        # Simple greedy selection: pick the first one (could be improved by distance)
        target = valid_frontiers[0]

        # Convert to world coordinates
        wx = self.map_info.origin.position.x + target[0] * self.map_info.resolution
        wy = self.map_info.origin.position.y + target[1] * self.map_info.resolution

        self.send_goal(wx, wy)

    def send_goal(self, x, y):
        self.get_logger().info(f'Sending goal to ({x}, {y})')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_namespace().lstrip('/') + '/map' if self.get_namespace() != '/' else 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.exploring = True
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.exploring = False
            return
            
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.exploring = False
        self.get_logger().info('Goal reached or aborted. Ready for next frontier.')

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
