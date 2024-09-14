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

The Funnyturtle project is divided into three main components:
1. Teleop Turtle

    In Turtle Teleop, the turtlesim_plus interface appears, allowing you to control the turtle's position and direction using the keys (a, s, d, w, etc.). Press 'i' to toggle the ability to spawn pizzas, 'o' to save the most recently spawned pizza to a .yaml file, and 'p' to clear the turtle's path so it can move directly to any unsaved pizza.

2. Copy Turtle Fleet

    In Copy Turtle Fleet, four turtles (Foxy, Noetic, Humble, and Iron) appear within the turtlesim_plus interface. This process begins automatically once the fourth teleop turtle completes saving. The Copy Turtle Fleet collects target points from the teleop pizza target .yaml file and uses these to spawn pizzas. Once all tasks are completed, the turtles move to the lower-left corner of the interface and wait for the next task.

3.  (Optional) Bonus features 

    After the Turtle teleop and the copy turtle fleet have been completed, all copy turtles will position themselves in the right corner. Then, spawn a turtle named "Melodic". Melodic will proceed to consume any pizzas and also eat all the copy turtles. Once there are no turtles left on the copy turtle interface, Melodic will switch to the teleop turtle interface. There, Melodic will try to eat all the pizzas and finally consume the teleop turtle. After that, the entire program will reset to its initial state.



## System Archtecture
<!-- ![alt text](<Screenshot from 2024-09-15 02-44-41.png>)
![alt text](<Screenshot from 2024-09-15 02-43-10.png>) -->
### Node
- Teleop turtle node
    - teleop_key_node :  This node receives keyboard input, processes the key press, and publishes the result to the teleop_scheduler_node, which then determines the specific task associated with the key pressed.

    - teleop_scheduler_node : Subscribes to the input key from the teleop_key_node and sends commands to define the turtle's state and actions, such as spawning pizza, clearing pizza, saving pizza positions, etc. Additionally, it manages pizza positions, either sending them to the controller_node when an unsaved pizza needs to be cleared or removed, or saving the pizza path positions to a .yaml file.

    - controller_node : This node uses a PID controller to minimize the distance between the turtle and the target position. Once the turtle is sufficiently close to the target, it notifies the teleop_scheduler_node that the task is complete and it is ready for the next task.

    - turtlesim_plus_node : Displays all components and interactions within the turtlesim_plus interface. Additionally, it sends the turtle's position and orientation to the controller_node and teleop_scheduler_node to be used for computations and task management.

- Copy turtle node
    - copy_scheduler_node : 

    - controller_node : This node uses a PID controller to minimize the distance between each turtle and the their target position. Once the turtle is sufficiently close to the target, it notifies the teleop_scheduler_node that the task is complete and it is ready for the next task.

    - turtlesim_plus_node

### Topic
- /cmd_vel : For command linear and angular velocity
- /target : Collect target position [x,y,z]
- /key : Collect button press from keyboard
- /pose : Position and Orientation of turtle

### Service
- /eat : For remove entity that are in area of scanner
- /spawn_pizza : For spawn pizza at specific position
- /notify : To acknowledge task done

### Parameter (Able to change with rqt interface)
- Controller Gain (Kp, Ki, Kd) :  Linear and angular controller gain
- Pizza_max : Maximum number of pizza that can spawn  

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
### Run seperately node

```bash
ros2 run funnyturtle {Node_name}
```

### Launch teleop turtle only
```bash
ros2 launch funnyturtle teleop.launch.py
```

### Launch teleop and copy tutles together
```bash
ros2 launch funnyturtle funnyturtle.launch.py
```


## Testing and Result

### Create "FIBO" by teleop turtle and automatic generate by a fleet of copy turtle