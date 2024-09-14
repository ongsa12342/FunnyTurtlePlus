#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import String
from turtlesim.msg import Pose
import numpy as np
        
# from std_srvs.srv import SetBool

class TeleopSchedulerNode(Node):
    def __init__(self):
        super().__init__('teleop_scheduler_node')

        # Subscriber for /key topic (char)
        self.create_subscription(
            String,
            '/key',
            self.key_callback,
            10
        )

        self.create_subscription(
            Pose, 
            "/turtle1/pose", 
            self.pose_callback, 
            10
        )

        # Publisher for /target (Point) and /cmd_vel (Twist)
        self.target_publisher = self.create_publisher(Point, '/target', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)


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
        self.turtle_pos = np.array([0.0,0.0,0.0])
        self.last_pizza_pos = np.array([0.0, 0.0])

    def pose_callback(self, msg):
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y
        self.turtle_pos[2] = msg.theta

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
        # elif self.current_key == 'c':
        #     self.state = 'Clear'
        #     self.get_logger().info("State changed to: Clear")
        # elif self.current_key == 'p':
        #     self.state = 'Spawn'
        #     self.get_logger().info("State changed to: Spawn")
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
            twist.linear.x = 0
            twist.angular.z = 0

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
            # self.get_logger().info("Saving current state...")  # You can add logic for saving state
            pass
            # Return to Idle after completing the Save action
            self.state = 'Idle'
        elif self.state == 'Clear':
            # self.get_logger().info("Clearing the saved state...")  # Add logic to clear
            pass
            # Return to Idle after completing the Clear action
            self.state = 'Idle'
        elif self.state == 'Spawn':
            # self.get_logger().info("Spawning an item...")  # Add logic to spawn
            self.turtle_teleop()
            self.spawn_pizza()


    def eat_callback(self, request, response):
        self.get_logger().info("Eating pizza")
        return Empty.Response()

    def notify_callback(self, request, response):
        self.get_logger().info(f"Notify request: {request.data}")
        response.data = True
        return response
    def spawn_pizza(self):
        # Check if the last pizza position is set
        if self.last_pizza_pos[0] is not None and self.last_pizza_pos[1] is not None:
            distance_moved = np.linalg.norm(self.turtle_pos[:2] - self.last_pizza_pos)

            # Spawn a pizza only if the distance moved is greater than or equal to 0.5
            if distance_moved < 0.5:
                return

        # Set the current position as the new pizza spawn position
        self.last_pizza_pos = self.turtle_pos[:2].copy()

        position_request = GivePosition.Request()
        position_request.x = self.turtle_pos[0]
        position_request.y = self.turtle_pos[1]
        self.spawn_pizza_client.call_async(position_request)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
