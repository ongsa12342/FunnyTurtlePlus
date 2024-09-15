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
from std_msgs.msg import Float64MultiArray
import time 
import subprocess
class copy_main_scheduler_node(Node):
    
    def __init__(self):
        super().__init__('copy_main_scheduler_node')
        
        # Define .YAML file path 
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'
        
        # Load the pizza paths from YAML when the node starts
        self.loaded_paths = None

        # Assign the first four paths to separate variables if available
        self.path = None
        self.pizza_path = None
        
        # # Parameters
        self.declare_parameter('turtle_name_list', ['turtle', 'turtlee'])

        # Get the parameter value (a list of strings)
        self.turtle_name_list = self.get_parameter('turtle_name_list').get_parameter_value().string_array_value

        # Log or use the parameter
        self.get_logger().info(f"Received string list: {self.turtle_name_list}")
        # # Subscribers


         # Create services for each turtle and pass the turtle name
        self.task_service = {}
        self.task_service_flags = {}
        for turtle_name in self.turtle_name_list:
            # Use a lambda to pass the turtle name to the callback
            self.task_service[turtle_name] = self.create_service(
                Notify,
                f'{turtle_name}/waiting',
                lambda request, response, turtle_name=turtle_name: self.noti_sent(request, response, turtle_name)
            )
            self.get_logger().info(f"Service created for {turtle_name}/waiting")

            # Initialize the flag for each turtle as False
            self.task_service_flags[turtle_name] = False

        self.archive = False

    def noti_sent(self, request: Notify.Request, response: Notify.Response, turtle_name: str):
        """Service callback that handles the Notify request, specific to each turtle."""
        
        # Log which turtle's flag is being set
        # self.get_logger().info(f"Received flag_request from {turtle_name}: {request.flag_request}")
        
        # Update the flag for this specific turtle
        self.task_service_flags[turtle_name] = request.flag_request
        
        # Check if all turtles have responded with True
        if all(self.task_service_flags.values()):
            # self.get_logger().info("All turtles have responded with flag_response=True.")
            self.archive = True  # Perform the next step or transition state
        else:
            # self.get_logger().info(f"Waiting for other turtles: {self.task_service_flags}")
            self.archive = False  # Continue waiting
        
        # Respond to the service
        response.flag_response = self.archive
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = copy_main_scheduler_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
