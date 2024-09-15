# FunnyTurtlePlus
## Introduction 
FunnyTurtle Project, FRA501 Robotics DevOps project. This project is designed to deepen your understanding of the basics of ROS2 (Robot Operating System 2) by offering practical experience in implementing key concepts such as Topics, Services, Parameters, Namespaces, and Launch Files within a complex robotic system. 

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture](#system-archtecture)
- [Installation and setup](#installation-and-setup)
- [Usage](#usage)
- [Testing and Result](#testing-and-result)

## Project Overview

The FunnyTurtlePlus project consists of three main components:

1. **Teleop Turtle**

   Control a turtle in the `turtlesim_plus` interface using your keyboard (W, A, S, D, etc.). Press:

   - `i` to toggle pizza spawning.
   - `o` to save the most recently spawned pizza to a `.yaml` file.
   - `p` to clear the turtle's path, allowing it to move directly to any unsaved pizza.

2. **Copy Turtle Fleet**

   After you've saved pizzas using the Teleop Turtle, four new turtles named Foxy, Noetic, Humble, and Iron will appear. They automatically read the saved pizza positions from the `.yaml` file and spawn pizzas at those locations. Once they've completed their tasks, they move to the lower-left corner and wait.

3. **Bonus Features (Optional)**

   Once the Copy Turtle Fleet is done, all turtles position themselves in the right corner. A new turtle named "Melodic" spawns and begins to "eat" all the pizzas and the Copy Turtles. After consuming everything in the Copy Turtle interface, Melodic moves to the Teleop Turtle interface to eat all pizzas and the Teleop Turtle itself. The program then resets to its initial state.


## System Archtecture
![alt text](<เต่าหัวกล้วย (2).png>)

### Nodes

- **teleop_key_node**: Captures keyboard input and publishes key presses to the scheduler.

- **teleop_scheduler_node**: Manages turtle actions like spawning pizzas, clearing paths, and saving pizza positions. Communicates with the controller for movement tasks.

- **controller_node**: Uses a PID controller to move the turtle towards target positions. Notifies the scheduler upon task completion.

- **turtlesim_plus_node**: Visualizes all turtles and pizzas. Sends position and orientation data to other nodes.

- **copy_scheduler_node**: Similar to the teleop scheduler but for the Copy Turtle Fleet. Loads target positions from the `.yaml` file.

- **copy_main_scheduler_node**: Coordinates the Copy Turtle Fleet, ensuring they complete tasks together and move to designated positions.

### Topics

- `/cmd_vel`: Sends movement commands to turtles.

- `/target`: Publishes target positions `[x, y, z]`.

- `/key`: Receives keyboard inputs.

- `/pose`: Provides the turtle's current position and orientation.

### Services

- `/eat`: Removes entities within a certain area.

- `/spawn_pizza`: Spawns a pizza at a specified location.

- `/notify`: Acknowledges task completion.

### Parameters (Adjustable via `rqt`)

- **Controller Gains (Kp, Ki, Kd)**: Adjusts the PID controller's responsiveness.

- **Pizza_max**: Sets the maximum number of pizzas that can be spawned.

## Installation and Setup

### Step 1: Clone the repository

```bash
git clone https://github.com/ongsa12342/FunnyTurtlePlus.git
```

### Step 2: Build the Package
```bash
cd FunnyTurtlePlus && colcon build
```
### Step 3: Build turtlesim plus
```bash
source dependencies_install.bash && colcon build --packages-select turtlesim_plus turtlesim_plus_interfaces
```
### Step 4: Source the Setup File
```bash
source ~/FunnyTurtlePlus/install/setup.bash
```
### Step 5: (Optional) Add to .bashrc
```bash
echo "source ~/FunnyTurtlePlus/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## Usage
- ### Run seperately node

```bash
    ros2 run funnyturtle {Node_name}
```

- ### Launch teleop turtle only
```bash
    ros2 launch funnyturtle teleop.launch.py
```

- ### Launch teleop and copy tutles together
```bash
    ros2 launch funnyturtle funnyturtle.launch.py
```

- ### Launch Extra Melodic turtle (After copy turtle was done!)
```bash
    ros2 launch funnyturtle melodic.launch.py
```

## Testing and Result

### Create "FIBO" by teleop turtle and automatic generate by a fleet of copy turtle
[![FunnyTurtlePlus Demo](<Screenshot from 2024-09-15 08-14-15.png>)](https://youtu.be/Kh8mGpidVNE)

### Extra Melodic turtle
[![FunnyTurtlePlus Demo](<Screenshot from 2024-09-15 08-17-30.png>)](https://youtu.be/hcOs1CRAHTI)

