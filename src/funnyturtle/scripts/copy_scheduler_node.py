#!/usr/bin/env python3
import fcntl
import rclpy
from rclpy.node import Node
import yaml
import os
from collections import deque
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from copy import deepcopy
from turtlesim_plus_interfaces.srv import GivePosition
from funnyturtleplus_interfaces.srv import Notify
import numpy as np

import time 
class copy_scheduler_node(Node):
    
    def __init__(self):
        super().__init__('copy_scheduler_node')
        
        # Define .YAML file path 
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'
        
        # Load the pizza paths from YAML when the node starts
        self.loaded_paths = None

        # Assign the first four paths to separate variables if available
        self.path = None
        self.pizza_path = None
        

        # Parameters
        self.declare_parameter('turtle_name', "turtle")
        self.turtle_name = self.get_parameter('turtle_name').value

        # Subscribers
        self.create_subscription(Pose, f"{self.turtle_name}/pose", self.pose_callback, 10)
        
        # Publishers
        self.target_publisher = self.create_publisher(Point, f'{self.turtle_name}/target', 10)
        
        # Timers
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Service Clients
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')

        # Service Servers
        self.task_service = self.create_service(Notify, f'{self.turtle_name}/noti', self.noti_sent)
        
        self.state = 'Idle'
        self.flag = True
        self.turtle_pos = np.array([0.0, 0.0])
        
        
    def pose_callback(self, msg):
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y
        
    def noti_sent(self, request: Notify.Request, response: Notify.Response):
        self.flag = request._flag_request
        self.get_logger().info(f"self flag, {self.flag}")
        if self.flag == True:
            self.spawn_pizza()
        return response
    
    def timer_callback(self):
        if self.state == 'Idle':
            self.loaded_paths = self.load_pizza_paths()
            
            if self.loaded_paths:
                if len(self.loaded_paths) == 4:
                    
                    self.path = self.assign_paths()
                    self.pizza_path = deque()
                    
                    self.path2array()
                    self.state = 'Spawn'
                    
        elif self.state == 'Spawn' :
            if self.flag == True:
                if len(self.pizza_path) > 0:
                    target = self.pizza_path.popleft()
                    print("Dequeued:", target)

                    # Create a Point message to represent the turtle's current position (for pizza)
                    target_position = Point()
                    target_position.x = target[0]
                    target_position.y = target[1]
                    target_position.z = 0.0

                    # Publish the target to the /target topic
                    self.target_publisher.publish(target_position)
                    self.flag = False
                else:
                    self.state = 'Idle'
        elif self.state == 'Waiting' :
            pass
        
    def spawn_pizza(self):

        position_request = GivePosition.Request()
        position_request.x = self.turtle_pos[0]
        position_request.y = self.turtle_pos[1]
        self.spawn_pizza_client.call_async(position_request)
        
    def load_pizza_paths(self):
        if os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)

            # if yaml_data is None:
            #     # Log a warning if the file is empty or invalid
            #     self.get_logger().warning("YAML file is empty or invalid.")
            #     return []
            loaded_paths = []  # To collect all loaded paths

            if 'PizzaPaths' in yaml_data:
                # Convert each saved path to a list of geometry_msgs.msg.Point objects
                for path in yaml_data['PizzaPaths']:
                    path_points = []
                    for point in path:
                        point_msg = Point()
                        point_msg.x = point[0]
                        point_msg.y = point[1]
                        point_msg.z = 0.0  # Assuming z is 0
                        path_points.append(point_msg)
                    loaded_paths.append(path_points)
            return loaded_paths
        else:
            # self.get_logger().info("No YAML file found.")
            return []

    def assign_paths(self):
        # Extract the first four paths if they exist, otherwise assign None
        path1 = self.loaded_paths[0] if len(self.loaded_paths) > 0 else None
        path2 = self.loaded_paths[1] if len(self.loaded_paths) > 1 else None
        path3 = self.loaded_paths[2] if len(self.loaded_paths) > 2 else None
        path4 = self.loaded_paths[3] if len(self.loaded_paths) > 3 else None
        
        return path1, path2, path3, path4
    
    def path2array(self):
        # Iterate over each path in self.path and log the result
        temp_path = []
        self.get_logger().info(f"turtle_name: {self.turtle_name}")
        for path in self.path:
            # Convert the path to a numpy array
            temp_path.append(np.array([[p.x, p.y] for p in path]))
            # Log as a numpy array
        
        self.path = temp_path
        # self.path = temp_path
        # time.sleep(3)
        # self.get_logger().info(f"temp_path: {temp_path}")
        # time.sleep(3)
        # self.get_logger().info(f"self.path: {self.path[0]}")
        # time.sleep(100)
        
        if self.turtle_name == 'Foxy':
            for pos in self.path[0]:
                self.pizza_path.append(deepcopy(pos))
        elif self.turtle_name == 'Noetic':
            for pos in self.path[1]:
                self.pizza_path.append(deepcopy(pos))
        elif self.turtle_name == 'Humble':
            for pos in self.path[2]:
                self.pizza_path.append(deepcopy(pos))
        elif self.turtle_name == 'Iron':
             for pos in self.path[3]:
                self.pizza_path.append(deepcopy(pos))
        # time.sleep(100)
        self.get_logger().info(f"path as numpy array: {self.pizza_path}")




def main(args=None):
    time.sleep(5)
    rclpy.init(args=args)
    node = copy_scheduler_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
