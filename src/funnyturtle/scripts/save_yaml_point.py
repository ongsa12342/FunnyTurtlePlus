import rclpy
from rclpy.node import Node
import yaml
import os
from geometry_msgs.msg import Point

class PointToYamlNode(Node):
    def __init__(self):
        super().__init__('point_to_yaml_node')
        # List of geometry_msgs/Point objects
        self.points = [Point(x=1.0, y=2.0), Point(x=3.0, y=4.0), Point(x=5.0, y=6.0)]  # Replace with actual points
        # Namespace
        self.namespace = 'sssss'  # Replace with your actual namespace
        # YAML file path
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'  # Replace with the actual file path
        # Write the points to a YAML file
        self.create_yaml_file()

    def create_yaml_file(self):
        # Convert the list of Point objects into the required format
        formatted_points = [
            {'x': point.x, 'y': point.y, 'z': point.z} for point in self.points
        ]

        # Create the dictionary structure to store in YAML
        yaml_dict = {
            f'/{self.namespace}/scheduler_node': {
                'ros__parameters': {
                    'target': formatted_points
                }
            }
        }

        # Write the YAML file
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(yaml_dict, yaml_file, default_flow_style=False)

        self.get_logger().info(f'YAML file created at {self.yaml_file_path}')

def main(args=None):
    rclpy.init(args=args)
    point_to_yaml_node = PointToYamlNode()
    rclpy.spin(point_to_yaml_node)

    # Shutdown the node
    point_to_yaml_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
