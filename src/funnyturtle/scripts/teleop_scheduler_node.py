#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import String
from turtlesim.msg import Pose
import numpy as np
from collections import deque
from funnyturtleplus_interfaces.srv import Notify
import yaml
import os
# from std_srvs.srv import SetBool

class TeleopSchedulerNode(Node):
    def __init__(self):
        super().__init__('teleop_scheduler_node')
        self.declare_parameter('turtle_name', "turtle")
        self.turtle_name = self.get_parameter('turtle_name').value
        self.task_service = self.create_service(Notify, f'{self.turtle_name}/noti', self.noti_sent)
        self.flag = True
        self.create_subscription(String, f'{self.turtle_name}/key', self.key_callback,  10)

        self.create_subscription(
            Pose, 
            f"{self.turtle_name}/pose", 
            self.pose_callback, 
            10
        )

        # Publisher for /target (Point) and /cmd_vel (Twist)
        self.target_publisher = self.create_publisher(Point, f'{self.turtle_name}/target', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 10)


        # Service for /spawn_pizza (GivePosition)
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')


        self.time_tolerance = rclpy.duration.Duration(seconds=0.05)

        self.linear_velocity = 10.0
        self.angular_velocity = 10.0

        # # Service for /notify (SetBool)
        # self.notify_service = self.create_service(
        #     SetBool,  # Use the correct service type
        #     '/notify',
        #     self.notify_callback
        # )

        # State machine states
        self.state = 'Idle'  # Possible states: 'Idle', 'Save', 'Clear', 'Spawn'
        self.current_key = ''
        self.last_key_time = self.get_clock().now()
        self.get_logger().info("TeleopSchedulerNode has been started.")

        # Timer to regularly check the state and act
        self.timer = self.create_timer(0.01, self.timer_callback)  
        self.turtle_pos = np.array([0.0,0.0])
        self.last_pizza_pos = np.array([0.0, 0.0])

        self.pizza_path = deque()

        self.client_eat = self.create_client(Empty, f'{self.turtle_name}/eat')
        
        self.save_count = 0
        
        self.max_saves = 4  # Save only up to 4 times
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'

        if os.path.exists(self.yaml_file_path):
            os.remove(self.yaml_file_path)
            print(f"File {self.yaml_file_path} has been removed.")
        else:
            print(f"File {self.yaml_file_path} does not exist.")


    def noti_sent(self, request:Notify.Request , response:Notify.Response):
        self.flag = request._flag_request
        self.get_logger().info(f"self flag, {self.flag}")
        return response
    
    def handle_save_state(self):
        if self.state == 'Save':
            # Increment the save count
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

                # Return to Idle after completing the Save action
                self.state = 'Idle'
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

        # Adjust linear and angular velocities based on key input
        if self.current_key == 'h':
            self.linear_velocity += 0.1  # Increase linear velocity
        elif self.current_key == 'n':
            self.linear_velocity = max(0.0, self.linear_velocity - 0.1)  # Decrease linear velocity, but not below 0
        elif self.current_key == 'j':
            self.angular_velocity += 0.1  # Increase angular velocity
        elif self.current_key == 'm':
            self.angular_velocity = max(0.0, self.angular_velocity - 0.1)  # Decrease angular velocity, but not below 0

        
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
            # Create and publish Twist messages based on the key pressed
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
                    
                    # Publish the target (turtle's current position) to the /target topic
                    self.target_publisher.publish(target_position)
                    self.flag = False
                else:
                    self.state = 'Idle'
            
            # # Display the queue after dequeuing
            # print("Queue after dequeuing:", list(self.pizza_path))
            
            # Return to Idle after completing the Clear action
            # self.state = 'Idle'
        elif self.state == 'Spawn':
            self.get_logger().info("Spawning an item...")  # Add logic to spawn
            self.turtle_teleop()
            self.spawn_pizza()

    def spawn_pizza(self):
        # Check if the last pizza position is set
        if self.last_pizza_pos[0] is not None and self.last_pizza_pos[1] is not None:
            distance_moved = np.linalg.norm(self.turtle_pos[:2] - self.last_pizza_pos)

            # Spawn a pizza only if the distance moved is greater than or equal to 0.5
            if distance_moved < 0.5:
                return

        # Set the current position as the new pizza spawn position
        self.last_pizza_pos = self.turtle_pos[:2].copy()

        self.pizza_path.append(self.turtle_pos.copy())

        position_request = GivePosition.Request()
        position_request.x = self.turtle_pos[0]
        position_request.y = self.turtle_pos[1]
        self.spawn_pizza_client.call_async(position_request)
        # self.get_logger().info(f"Pizza Path : {self.pizza_path}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
