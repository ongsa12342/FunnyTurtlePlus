from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    launch_description = LaunchDescription()  # Correct variable naming

    # Define namespaces and turtle spawn data
    namespaces = ['teleop', 'copy']
    turtle_name = 'melodic'

    # Spawn turtle service call
    spawn_turtle = ExecuteProcess(
        cmd=[f"ros2 service call /{namespaces[1]}/spawn_turtle turtlesim/srv/Spawn \"{{x: 10.7, y: 10.7, theta: 6.28, name: '{turtle_name}'}}\""],
        shell=True
    )
    launch_description.add_action(spawn_turtle)

    

    # Launch teleop_key_node with namespace remapping
    teleop_key_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'funnyturtle', 'melodic.py',
            '--ros-args', '--remap', f'__ns:=/{namespaces[1]}',
            '--param', f'turtle_name:={turtle_name}'
        ],
        output='screen',
        shell=True  # Adding shell=True for proper command execution
    )
    launch_description.add_action(teleop_key_node)

    # Launch teleop_controller_node.py
    controller_node = Node(
        package='funnyturtle',
        executable='controller_node.py',
        namespace=namespaces[1],
        name='controller_node',
        parameters=[{'turtle_name': turtle_name}, 
                    {'Kp_linear': 4.0},
                    {'Kp_angular': 5.0},
                    {'Kd_angular': 5.0} ] 
    )
    launch_description.add_action(controller_node)

    return launch_description
