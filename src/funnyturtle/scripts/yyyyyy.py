import rclpy
from rclpy.node import Node
import yaml
from pathlib import Path

class YamlCreatorNode(Node):
    def __init__(self):
        super().__init__('yaml_creator_node')
        self.get_logger().info("YamlCreatorNode has started.")
        
        # Create YAML file on node startup
        self.create_yaml_file()

    def create_yaml_file(self):
        # Define your YAML content here
        data = {
            'name': 'pizza',
            'positions': [
                {'x': 1.0, 'y': 1.0, 'theta': 0.0},
                {'x': 2.0, 'y': 3.0, 'theta': 1.57}
            ]
        }

        # Define the exact file path
        file_path = Path('/home/farao/FunnyTurtlePlus/src/funnyturtle/config/Pizzapath.yaml')

        # Create parent directories if they don't exist
        file_path.parent.mkdir(parents=True, exist_ok=True)

        # Check if the file already exists and log a message
        if file_path.exists():
            self.get_logger().info(f"File {file_path} already exists and will be replaced.")

        # Write data to the YAML file (overwrite if exists)
        with open(file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

        self.get_logger().info(f"YAML file created/replaced at: {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = YamlCreatorNode()

    # Keep node alive
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
