#!/usr/bin/python3

from funnyturtle.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose
from funnyturtle.pid_controller import PIDController
import numpy as np
from funnyturtleplus_interfaces.srv import Notify
import math
from std_srvs.srv import Empty


class teleop_controller_node(Node):
    def __init__(self):
        super().__init__('teleop_controller_node')
        # Declare parameters with default values
        self.declare_parameter('Kp_linear', 0.5)
        self.declare_parameter('Ki_linear', 0.0)
        self.declare_parameter('Kd_linear', 0.0)
        self.declare_parameter('U_max_linear', 100.0)
        
        self.declare_parameter('Kp_angular', 0.5)
        self.declare_parameter('Ki_angular', 0.0)
        self.declare_parameter('Kd_angular', 0.0)
        self.declare_parameter('U_max_angular', 100.0)

        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Ki_linear = self.get_parameter('Ki_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.U_max_linear = self.get_parameter('U_max_linear').value
        
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Ki_angular = self.get_parameter('Ki_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        self.U_max_angular = self.get_parameter('U_max_angular').value
        
        
        
        
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)


        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.create_subscription(Point, "target_position", self.target_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.flag_client = self.create_client(Notify, 'noti')
        self.eat_pizza_cli = self.create_client(Empty, 'eat')


        self.pid_angular = PIDController(self.Kp_angular, self.Ki_angular, self.Kd_angular, self.U_max_angular)
        self.pid_linear = PIDController(self.Kp_linear, self.Ki_linear, self.Kd_linear, self.U_max_linear)
        
        self.target_pos = np.array([0.0, 0.0])
        self.turtle_pos = np.array([0.0, 0.0, 0.0])
        self.threshold = 0.1
        
        self.add_on_set_parameters_callback(self.set_param_callback)
        
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                self.get_logger().info(f'Updated Kp: {param.value}')
                self.Kp = param.value
            elif param.name == 'Ki':
                self.get_logger().info(f'Updated Ki: {param.value}')
                self.Ki = param.value
            elif param.name == 'Kd':
                self.get_logger().info(f'Updated Kd: {param.value}')
                self.Kd = param.value
            elif param.name == 'U_max':
                self.get_logger().info(f'Updated U_max: {param.value}')
                self.U_max = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        self.pid_angular.set_param(self.Kp_angular, self.Ki_angular, self.Kd_angular, self.U_max_angular)
        self.pid_linear.set_param(self.Kp_linear, self.Ki_linear, self.Kd_linear, self.U_max_linear)
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
        
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_cli.call_async(eat_request)
        
    def pose_callback(self, msg):
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y
        self.turtle_pos[2] = msg.theta
        # self.get_logger().info(f"turtle pose, {self.turtle_pos}")

        

    def target_callback(self, msg):
        self.target_pos[0] = msg.x
        self.target_pos[1] = msg.y
        # self.get_logger().info(f"target, {self.target_pos}")

    
    def timer_callback(self):
        dt = 1.0 / self.freq  # Time step

        # Calculate distance and angle to the target
        distance = np.linalg.norm(self.target_pos - self.turtle_pos[:2])
        desired_theta = math.atan2(self.target_pos[1] - self.turtle_pos[1], self.target_pos[0] - self.turtle_pos[0])
        theta_error = desired_theta - self.turtle_pos[2]

        # Normalize the angle error to [-pi, pi]
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        if distance < self.threshold:
            # self.cmdvel(0.0, 0.0)  # Stop the turtle
            self.data = Notify.Request()
            self.data.flag_request = True
            self.flag_client.call_async(self.data)
            self.get_logger().info(f"data !!!, {self.data}")
            return
        
        # Get control signals from PID using compute method
        angular_control = self.pid_angular.compute(theta_error)
        linear_control = self.pid_linear.compute(distance)

        # Send velocity commands
        self.cmdvel(linear_control, angular_control)




def main(args=None):
    rclpy.init(args=args)
    node = teleop_controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
