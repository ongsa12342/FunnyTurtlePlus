from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    namespace = 'teleop'
    turtlename = 'turtle_teleop'
    return LaunchDescription([
        # Launch turtlesim_plus_node.py with namespace 'turtle1'
        Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node.py',
            namespace=namespace,
            name='turtlesim_plus_node',
            parameters=[{'turtle_name': turtlename}]
        ),
        
        # Launch teleop_controller_node.py
        Node(
            package='funnyturtle',
            executable='teleop_controller_node.py',
            namespace=namespace,
            name='teleop_controller_node',
            parameters=[{'turtle_name': turtlename}]
        ),
        
        # Launch teleop_scheduler_node.py
        Node(
            package='funnyturtle',
            executable='teleop_scheduler_node.py',
            namespace=namespace,
            name='teleop_scheduler_node',
            parameters=[{'turtle_name': turtlename}]
        ),
        
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'funnyturtle', 'teleop_key_node.py',
                 '--ros-args', '--remap', '__ns:=/teleop', '-p', 'turtle_name:=turtle_teleop'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=["ros2 service call /teleop/remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
            shell=True
        ),

        ExecuteProcess(
                cmd=[f"ros2 service call /teleop/spawn_turtle turtlesim/srv/Spawn \"{{x: -1.0, y: -1.0, theta: 0.0, name: {turtlename}}}\""],
                shell=True
            ),
    ])
