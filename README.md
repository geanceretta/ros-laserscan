# ros-laserscan
object detection using laser scan, create a alternative route.

## usage

First you will create a empty world with turtlebot3, than run the control script position_control_TB3.py

### Create the empty world
Inside your ~/catkin_ws directory run:

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```bash

Inside gazebo, put some obstacles for the robot to avoid

### Run the control script 
From your ~/catkin_ws directory run:

```
python src/rm_exemplos/src/scripts/position_control_TB3.py
```bash

When prompted, set the position where the robot should arrive. Think of a path where the object avoidance is needed.

Console will print some information about the robot decisions. It will end when arive.

## Known issues
Sometimes the robot can start spinning on its own axel, so you can place it somewhere else, set coordinates to a far point or restart the simulation.
