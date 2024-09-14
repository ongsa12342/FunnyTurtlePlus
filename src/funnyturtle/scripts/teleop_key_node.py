#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses

class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key_node')
        
        self.declare_parameter('turtle_name', "turtle")
        self.turtle_name = self.get_parameter('turtle_name').value
        # Publisher for /key topic (String)
        self.key_publisher = self.create_publisher(String, f'{self.turtle_name}/key', 10)
        self.get_logger().info("TeleopKeyNode has been started.")

    def run(self):
        curses.wrapper(self.curses_loop)

    def curses_loop(self, stdscr):
        # Set up curses settings
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.clear()

        # Create the title and header for the menu
        stdscr.addstr(1, 2, "TurtleSim Control Menu", curses.A_BOLD | curses.A_UNDERLINE)

        # Create the instructions section with better formatting
        stdscr.addstr(3, 4, "Control the turtle using the following keys:", curses.A_BOLD)
        
        # Control instructions with better alignment and spacing
        stdscr.addstr(5, 6, "w/s   : Move forward/backward")
        stdscr.addstr(6, 6, "a/d   : Rotate left/right")
        stdscr.addstr(7, 6, "h/n   : Increase/Decrease moving speed")
        stdscr.addstr(8, 6, "j/m   : Increase/Decrease rotating speed")
        stdscr.addstr(9, 6, "space : Force stop")
        stdscr.addstr(10, 6, "i     : Spawn pizza")
        stdscr.addstr(11, 6, "o     : Save pizza path")
        stdscr.addstr(12, 6, "p     : Clear pizza path")
        stdscr.addstr(13, 6, "q     : Quit")

        # Refresh display initially
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()
            
            if key != -1:
                if key == ord('q'):
                    # Quit the application when 'q' is pressed
                    self.get_logger().info("Quitting...")
                    rclpy.shutdown()
                    break  # Exit the loop
                else:
                    self.publish_key(chr(key))

            # Spin once to process ROS callbacks without blocking
            rclpy.spin_once(self, timeout_sec=0)

    def publish_key(self, key):
        msg = String()
        msg.data = key

        self.key_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
