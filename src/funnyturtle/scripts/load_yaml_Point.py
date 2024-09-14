import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point

class YamlLoaderNode(Node):
    def __init__(self):
        super().__init__('yaml_loader_node')

        # YAML file path
        self.yaml_file_path = '/home/farao/FunnyTurtlePlus/src/funnyturtle/config/Pizzapath.yaml'  # Replace with your actual file path

        # Load the YAML file
        self.load_yaml_file()

    def load_yaml_file(self):
        # Load the YAML file
        with open(self.yaml_file_path, 'r') as yaml_file:
            yaml_content = yaml.safe_load(yaml_file)

        # Extract the list of points from the YAML file
        namespace = 'sssss'  # Replace with your actual namespace
        points_list = yaml_content[f'/{namespace}/scheduler_node']['ros__parameters']['target']

        # Print each point as a geometry_msgs/Point
        for point_data in points_list:
            point = Point(x=point_data['x'], y=point_data['y'], z=point_data['z'])
            self.print_point(point)

    def print_point(self, point):
        # Print the point's coordinates
        self.get_logger().info(f'Point: x={point.x}, y={point.y}')

def main(args=None):
    rclpy.init(args=args)
    yaml_loader_node = YamlLoaderNode()
    rclpy.spin(yaml_loader_node)

    # Shutdown the node
    yaml_loader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
