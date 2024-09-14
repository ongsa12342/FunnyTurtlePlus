# FunnyTurtlePlus

## Introduction 
Welcome to the FunnyTurtle Project, a hands-on initiative developed for the FRA501 Robotics DevOps course. This project is designed to deepen your understanding of the basics of ROS2 (Robot Operating System 2) by offering practical experience in implementing key concepts such as Topics, Services, Parameters, Namespaces, and Launch Files within a complex robotic system. 

## Project Overview

The Funnyturtle project is divided into two main components:
1. Teleop Turtle

The Teleop Turtle is a user-controlled robot that responds to keyboard inputs, allowing you to navigate and interact within the simulation environment. Its key functionalities include:

- Movement Control: Use keyboard keys (e.g., 'w', 'a', 's', 'd') to move the turtle in different directions.
- Pizza Spawning: Press a designated key to drop a pizza at the turtle's current location.
- Position Saving: Save the positions of all unsaved pizzas to a .yaml file by pressing a specific key. You can save up to four sets of positions.
- Clearing Pizzas: Remove all unsaved pizzas from the environment with a dedicated key press.
- Dynamic Configuration: Adjust parameters such as the number of pizzas (n) that can be spawned and controller gains via rqt.
- Namespace Support: The Teleop Turtle can be assigned a namespace for better organization and management within the ROS2 ecosystem.

2. Copy Turtle Fleet

Once you have saved pizza positions four times, an autonomous fleet of four turtles is activated. Each turtle in the fleet has a unique role:

- Turtle "Foxy":    Replicates the first set of saved pizza positions.
- Turtle "Noetic":  Replicates the second set of saved positions.
- Turtle "Humble":  Replicates the third set.
- Turtle "Iron":    Replicates the fourth set.

The fleet starts from the bottom-left corner and performs the following tasks:

1. Pizza Replication: Each turtle moves to the saved positions and drops pizzas accordingly.
2. Synchronization: After completing their tasks, all turtles wait until every member of the fleet is finished.
3. Final Movement: Together, they move to the top-right corner of the simulation environment.


## System Archtecture
- Topic
- Service
- Parameter
- Namespace
- Launch File

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

```bash
ros2 run funnyturtle {Node_name}
```
Run specific node 

```bash
ros2 launch funnyturtle teleop.launch.py
```
Launch all teleop turtle node (teleop_key_node, teleop_scheduler_node, controller_node) 

```bash
ros2 launch funnyturtle funnyturtle.launch.py
```
Launch turtle teleop and copy turtle together 


## Features and Functionality
- Turtle teleop 

    In Turtle Teleop, the turtlesim_plus interface appears, allowing you to control the turtle's position and direction using the keys (a, s, d, w, etc.). Press 'i' to toggle the ability to spawn pizzas, 'o' to save the most recently spawned pizza to a .yaml file, and 'p' to clear the turtle's path so it can move directly to any unsaved pizza.

- Copy turtle fleet

    In Copy Turtle Fleet, four turtles (Foxy, Noetic, Humble, and Iron) appear within the turtlesim_plus interface. This process begins automatically once the fourth teleop turtle completes saving. The Copy Turtle Fleet collects target points from the teleop pizza target .yaml file and uses these to spawn pizzas. Once all tasks are completed, the turtles move to the lower-left corner of the interface and wait for the next task.



- Bonus features
## Testing and Result