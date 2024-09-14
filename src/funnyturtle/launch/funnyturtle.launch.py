from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    Launch_description = LaunchDescription()

    # Define namespaces and turtle spawn data
    namespaces = ['teleop', 'copy']
    turtles_to_spawn = {
        namespaces[0]: [
            {'turtlename': 'turtle_teleop', 'x': -1.0, 'y': -1.0, 'theta': 0.0}
        ],
        namespaces[1]: [
            {'turtlename': 'Foxy', 'x': -1.0, 'y': -1.0, 'theta': 0.0},
            {'turtlename': 'Noetic', 'x': -1.0, 'y': -1.0, 'theta': 0.0},
            {'turtlename': 'Humble', 'x': -1.0, 'y': -1.0, 'theta': 0.0},
            {'turtlename': 'Iron', 'x': -1.0, 'y': -1.0, 'theta': 0.0},
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

            # Launch teleop_controller_node.py
            controller_node = Node(
                package='funnyturtle',
                executable='controller_node.py',
                namespace=ns,
                name='controller_node',
                parameters=[{'turtle_name': turtle["turtlename"]}]
            )
            Launch_description.add_action(controller_node)

            # Launch teleop_scheduler_node.py
            scheduler_node = Node(
                package='funnyturtle',
                executable=ns+'_scheduler_node.py',
                namespace=ns,
                name=f'{ns}_scheduler_node',
                parameters=[{'turtle_name': turtle["turtlename"]}]
            )
            Launch_description.add_action(scheduler_node)

            # print()
            # print(f"{{x: {turtle['x']}, y: {turtle['y']}, theta: {turtle['theta']}, name: '{turtle['turtlename']}'}}\"")
            # print()
            spawn_turtle = ExecuteProcess(
                cmd=[f"ros2 service call /{ns}/spawn_turtle turtlesim/srv/Spawn \"{{x: {turtle['x']}, y: {turtle['y']}, theta: {turtle['theta']}, name: '{turtle['turtlename']}'}}\""],
                shell=True
            )
            Launch_description.add_action(spawn_turtle)

    teleop_key_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'funnyturtle', 'teleop_key_node.py',
                '--ros-args', '--remap', f'__ns:=/{namespaces[0]}', '-p', 'turtle_name:=turtle_teleop'],
        output='screen'
    )
    Launch_description.add_action(teleop_key_node)

    return Launch_description
