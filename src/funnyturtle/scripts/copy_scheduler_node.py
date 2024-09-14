import rclpy
from rclpy.node import Node
import yaml
import os
from geometry_msgs.msg import Point

class copy_scheduler_node(Node):
    
    def __init__(self):
        super().__init__('copy_scheduler_node')
        self.yaml_file_path = 'src/funnyturtle/config/Pizzapath.yaml'
        
        # Load the pizza paths from YAML when the node starts
        self.loaded_paths = self.load_pizza_paths()

        # Assign the first four paths to separate variables if available
        self.path1, self.path2, self.path3, self.path4 = self.assign_paths()

    def load_pizza_paths(self):
        if os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
            
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
            self.get_logger().info("No YAML file found.")
            return []

    def assign_paths(self):
        # Extract the first four paths if they exist, otherwise assign None
        path1 = self.loaded_paths[0] if len(self.loaded_paths) > 0 else None
        path2 = self.loaded_paths[1] if len(self.loaded_paths) > 1 else None
        path3 = self.loaded_paths[2] if len(self.loaded_paths) > 2 else None
        path4 = self.loaded_paths[3] if len(self.loaded_paths) > 3 else None
        return path1, path2, path3, path4


def main(args=None):
    rclpy.init(args=args)
    node = copy_scheduler_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
