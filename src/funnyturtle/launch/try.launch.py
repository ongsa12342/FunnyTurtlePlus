from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    Launch_description = LaunchDescription()

    # Define namespaces and turtle spawn data
    namespaces = ['teleop', 'copy']
    turtles_to_spawn = {
        'teleop': [
            {'name': 'teleop_turtle', 'x': 1.0, 'y': 1.0, 'theta': 0.0}
        ],
        'copy': [
            {'name': 'xxxx', 'x': 2.0, 'y': 2.0, 'theta': 0.0},
            {'name': 'yyyy', 'x': 5.0, 'y': 5.0, 'theta': 0.0},
            {'name': 'zzzz', 'x': 3.0, 'y': 5.0, 'theta': 0.0},
            {'name': 'asda', 'x': 8.0, 'y': 6.0, 'theta': 0.0}
        ]
    }

    for ns in namespaces:
        # Launch turtlesim_plus_node for each namespace
        turtle_sim_node = Node(
            package='turtlesim_plus',
            namespace=ns,
            executable='turtlesim_plus_node.py',
            name='turtle_sim'
        )
        Launch_description.add_action(turtle_sim_node)

        # Kill the default turtle1 in each namespace
        kill_turtle = ExecuteProcess(
            cmd=[f"ros2 service call /{ns}/remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
            shell=True
        )
        Launch_description.add_action(kill_turtle)

        # Spawn turtles as defined in turtles_to_spawn dictionary
        for turtle in turtles_to_spawn[ns]:
            spawn_turtle = ExecuteProcess(
                cmd=[f"ros2 service call /{ns}/spawn_turtle turtlesim/srv/Spawn \"{{x: {turtle['x']}, y: {turtle['y']}, theta: {turtle['theta']}, name: '{turtle['name']}'}}\""],
                shell=True
            )
            Launch_description.add_action(spawn_turtle)

    return Launch_description
