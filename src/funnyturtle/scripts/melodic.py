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
from std_srvs.srv import Empty


import time 
class melodic_node(Node):
    
    def __init__(self):
        super().__init__('melodic_node')
        
        # Define .YAML file path 
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'
        self.loaded_paths = None
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
        self.eat_pizza_cli = self.create_client(Empty, f'{self.turtle_name}/eat')


        # Service Servers
        self.task_service = self.create_service(Notify, f'{self.turtle_name}/noti', self.noti_sent)
        
        self.state = 'Idle'
        self.flag = True
        self.turtle_pos = np.array([0.0, 0.0])
        
        self.archive = False
        
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_cli.call_async(eat_request)
        
    def pose_callback(self, msg):
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y
        
    def noti_sent(self, request: Notify.Request, response: Notify.Response):
        self.flag = request._flag_request
        # self.get_logger().info(f"self flag, {self.flag}")
        self.archive = True
        return response
    
    def timer_callback(self):
        if self.state == 'Idle':
            self.loaded_paths = self.load_pizza_paths()
            if self.loaded_paths and len(self.loaded_paths) == 4:
                self.path = self.assign_paths()
                self.pizza_path = deque()
                self.path2array()
                self.state = 'Eat'
        elif self.state == 'Eat':
            self.eat_pizza()
            if self.flag:
                self.get_logger().info("Flag is True")
                if self.pizza_path:
                    self.get_logger().info("Pizza path deque is not empty")
                    target = self.pizza_path.popleft()  # Only call popleft if deque is not empty
                    self.get_logger().info(f"Dequeued target: {target}")
                    
                    target_position = Point()
                    target_position.x = target[0]
                    target_position.y = target[1]
                    target_position.z = 0.0

                    # Publish the target to the /target topic
                    self.target_publisher.publish(target_position)
                    self.flag = False
                else:
                    self.get_logger().info("Pizza path deque is empty, no more targets.")
            

            
            
    def load_pizza_paths(self):
        if os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)

            if yaml_data is None:
                # Log a warning if the file is empty or invalid
                self.get_logger().warning("YAML file is empty or invalid.")
                return []
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
        self.pizza_path.append(np.array([10.0,10.0]))
        self.pizza_path.append(np.array([5.0,5.0]))
        for pos in self.path[0]:
            self.pizza_path.append(deepcopy(pos))
        for pos in self.path[1]:
            self.pizza_path.append(deepcopy(pos))
        for pos in self.path[2]:
            self.pizza_path.append(deepcopy(pos))
        for pos in self.path[3]:
            self.pizza_path.append(deepcopy(pos))

        self.get_logger().info(f"path as numpy array: {self.pizza_path}")




def main(args=None):

    rclpy.init(args=args)
    node = melodic_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
