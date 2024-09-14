#!/usr/bin/python3

import os
from collections import deque

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

from turtlesim_plus_interfaces.srv import GivePosition
from funnyturtleplus_interfaces.srv import Notify

from rcl_interfaces.msg import SetParametersResult

class TeleopSchedulerNode(Node):
    def __init__(self):
        super().__init__('teleop_scheduler_node')

        # Parameters
        self.declare_parameter('turtle_name', "turtle")
        self.turtle_name = self.get_parameter('turtle_name').value

        self.declare_parameter('pizza_max', 20)
        self.pizza_max = self.get_parameter('pizza_max').value

        # Flags and Counters
        self.flag = True # controller notify
        self.save_count = 0
        self.max_saves = 4  # Save only up to 4 times
        self.pizza_count = 0

        # File paths
        self.yaml_file_path = 'src/funnyturtle/config/temp.yaml'
        self.yaml_finish_file_path = 'src/funnyturtle/config/Pizzapath.yaml'
        
        # Remove existing YAML file if it exists
        if os.path.exists(self.yaml_file_path):
            os.remove(self.yaml_file_path)
            print(f"File {self.yaml_file_path} has been removed.")
        else:
            print(f"File {self.yaml_file_path} does not exist.")
            
        if os.path.exists(self.yaml_finish_file_path):
            os.remove(self.yaml_finish_file_path)
            print(f"File {self.yaml_finish_file_path} has been removed.")
        else:
            print(f"File {self.yaml_finish_file_path} does not exist.")

        # State machine states
        self.state = 'Idle'  # Possible states: 'Idle', 'Save', 'Clear', 'Spawn'
        self.current_key = ''
        self.last_key_time = self.get_clock().now()
        self.time_tolerance = rclpy.duration.Duration(seconds=0.05)

        # Velocities
        self.linear_velocity = 10.0
        self.angular_velocity = 10.0

        # Positions and Paths
        self.turtle_pos = np.array([0.0, 0.0])
        self.pizza_path = deque()

        # Subscribers
        self.create_subscription(String, f'{self.turtle_name}/key', self.key_callback, 10)
        self.create_subscription(Pose, f"{self.turtle_name}/pose", self.pose_callback, 10)

        # Publishers
        self.target_publisher = self.create_publisher(Point, f'{self.turtle_name}/target', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 10)

        # Service Clients
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        self.client_eat = self.create_client(Empty, f'{self.turtle_name}/eat')

        # Service Servers
        self.task_service = self.create_service(Notify, f'{self.turtle_name}/noti', self.noti_sent)

        # Timers
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialization Logs
        self.get_logger().info("TeleopSchedulerNode has been started.")

        

        # Parameter Update Callback
        self.add_on_set_parameters_callback(self.set_param_callback)

    def set_param_callback(self, params):
        for param in params:
            if param.name == 'pizza_max':
                self.get_logger().info(f'Updated pizza_max: {param.value}')
                self.pizza_max = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                return SetParametersResult(
                    successful=False, reason=f'Unknown parameter: {param.name}'
                )
        return SetParametersResult(successful=True)

    def noti_sent(self, request: Notify.Request, response: Notify.Response):
        self.flag = request._flag_request
        self.get_logger().info(f"self flag, {self.flag}")
        return response

    def handle_save_state(self):
        if self.state == 'Save':
            if self.save_count < self.max_saves:
                self.get_logger().info(f"Pizza Path : {self.pizza_path}")

                # Convert deque and numpy arrays to a list of lists
                pizza_path_list = [point.tolist() for point in self.pizza_path]

                # Load existing data from the YAML file if it exists
                if os.path.exists(self.yaml_file_path):
                    with open(self.yaml_file_path, 'r') as file:
                        yaml_data = yaml.safe_load(file) or {}
                else:
                    yaml_data = {}

                # Append new pizza path to the existing list of paths
                if 'PizzaPaths' not in yaml_data:
                    yaml_data['PizzaPaths'] = []

                yaml_data['PizzaPaths'].append(pizza_path_list)

                # Save the updated data to the YAML file
                with open(self.yaml_file_path, 'w') as file:
                    yaml.dump(yaml_data, file, default_flow_style=False)

                self.pizza_path.clear()

                # Increment the save count
                self.save_count += 1
                self.get_logger().info(f"Save Count: {self.save_count}")
                self.pizza_count = 0
                # Return to Idle after completing the Save action
                self.state = 'Idle'
                if self.save_count == 4:
                    os.rename(self.yaml_file_path, self.yaml_finish_file_path)
            else:
                self.get_logger().info("Reached maximum save count. No more saves.")
                self.state = 'Idle'

    def pose_callback(self, msg):
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y

    def key_callback(self, msg):
        self.current_key = msg.data
        self.last_key_time = self.get_clock().now()
        self.get_logger().info(f"Received key: {self.current_key}")

        # Logic to change state based on key input
        if self.state == 'Idle' and self.current_key == 'i':
            self.state = 'Spawn'
            self.get_logger().info("State changed to: Spawn")
        elif self.state == 'Spawn' and self.current_key == 'i':
            self.state = 'Idle'
            self.get_logger().info("State changed to: Spawn")
        elif self.state == 'Idle' and self.current_key == 'p':
            self.state = 'Clear'
            self.get_logger().info("State changed to: Clear")
        elif self.current_key == 'o':
            self.state = 'Save'
            self.get_logger().info("State changed to: Spawn")
        # else:
        #     self.state = 'Idle'
        #     self.get_logger().info("State changed to: Idle")

    def turtle_teleop(self):
        twist = Twist()

        # Movement commands
        if self.current_key == 'w':
            twist.linear.x = self.linear_velocity  # Move forward
        elif self.current_key == 's':
            twist.linear.x = -self.linear_velocity  # Move backward
        elif self.current_key == 'a':
            twist.angular.z = self.angular_velocity  # Rotate left
        elif self.current_key == 'd':
            twist.angular.z = -self.angular_velocity  # Rotate right
        elif self.current_key == ' ':  # Space to stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Adjust velocities
        if self.current_key == 'h':
            self.linear_velocity += 0.1  # Increase linear velocity
        elif self.current_key == 'n':
            self.linear_velocity = max(0.0, self.linear_velocity - 0.1)  # Decrease linear velocity
        elif self.current_key == 'j':
            self.angular_velocity += 0.1  # Increase angular velocity
        elif self.current_key == 'm':
            self.angular_velocity = max(0.0, self.angular_velocity - 0.1)  # Decrease angular velocity

        # Reset twist if key timeout
        if (self.get_clock().now() - self.last_key_time) > self.time_tolerance:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)

        self.cmd_vel_publisher.publish(twist)

    def timer_callback(self):
        # Perform actions based on the current state
        if self.state == 'Idle':
            # self.get_logger().info("State is Idle. Waiting for input...")
            self.turtle_teleop()

        elif self.state == 'Save':
            self.handle_save_state()

        elif self.state == 'Clear':
            self.client_eat.call_async(Empty.Request())
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
                    self.pizza_count = 0
                    self.state = 'Idle'

            # # Display the queue after dequeuing
            # print("Queue after dequeuing:", list(self.pizza_path))

        elif self.state == 'Spawn':
            # self.get_logger().info("Spawning an item...")
            self.turtle_teleop()
            self.spawn_pizza()

    def spawn_pizza(self):
        if self.pizza_path:
            pizza_positions = np.array(self.pizza_path)  # Convert deque to numpy array
            distances = np.linalg.norm(pizza_positions - self.turtle_pos[:2], axis=1)

            # Check if any distance is smaller than 0.5
            if np.any(distances < 0.5):
                return
        
        if self.pizza_count >= self.pizza_max:
            self.get_logger().info(f"pizza = {self.pizza_count} then cannot spawn more")
            return
        self.pizza_path.append(self.turtle_pos.copy())

        position_request = GivePosition.Request()
        position_request.x = self.turtle_pos[0]
        position_request.y = self.turtle_pos[1]

        self.spawn_pizza_client.call_async(position_request)
        self.pizza_count += 1
        # self.get_logger().info(f"Pizza Path : {self.pizza_path}")
        self.get_logger().info(f"pizza = {self.pizza_count} ")



def main(args=None):
    rclpy.init(args=args)
    node = TeleopSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
