Synopsis
========
This project implements a Software system for navigation and frontier based exploration for mobile robotic platforms (Turtlebots).

Description
========
Description: First of all, Turtlebots are small robots that can drive around and sense the environment through a Kinect sensor.  
One of them is shown below.

![Turtlebot](https://github.com/bnurbekov/FrontierBasedExplorationAndNavigation/blob/master/Turtlebot.jpg){: width=250px}

In general, the purpose of the project was to build an informed search algorithm on a grid (shown below), so that the robot could explore the environment. 
The robot had to be able to locate borders of the unexplored zones (shown in orange) and find a path to those borders using an A* search. After that, the goal was to drive to the borders in order to explore those zones by spinning in one place.

![Exploration](https://github.com/bnurbekov/FrontierBasedExplorationAndNavigation/blob/master/Exploration.png){: width=250px}
(Explored cells are shown in white; expanded obstacles are shown in black; unexplored zone borders are shown in orange)

The project is interesting from the software engineering stand-point because it is very high-level (no low-level robotics involved), allowing to practice search algorithms, such as BFS, DFS and A*, and performance optimization techniques, such as multi-threading.  

Outcome: The robot was able to successfully explore the environment.

Installation
============
Save this package into catkin_ws/src/.

Run gazebo simulation by running 'roslaunch turtlebot_gazebo turtlebot_world.launch' or bringing up the actual turtlebot.

In a new tab, run 'rosrun rviz rviz'.

Open the final.rviz settings located in the 'rviz' folder.

Then run the following commands:
1) Run 'roslaunch final_project final_project.launch'
2) Run 'rosrun final_project mapping.py'. This will run the mapping service.
3) Run 'rosrun final_project control.py'. This will run the control script.

