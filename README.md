# Controlling-Husky-Robot
This repository is created to maintain a code in C++ language to control the motion of Husky Robot using ROS (Robot Operating System)

# About the node
The "controlHusky" node controls the motion of the Husky robot. Once the user feeds in the destination co-ordinates, the robot finds its path to it. The robot comes to a complete halt when it it within 0.5 meter proximity of the destination. The stopping position is always directed in the X directional axis. 

# How to install the node?
1) Make sure you have "husky_simulator" installed on your ROS. Follow ROS tutorial for CPP at http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
2) Create a folder with name "deepfield_task" in the src directory of your catkin workspace. This folder represents as package name. Clone the main branch of this repository into the newly created "deepfield_task" directory of your catkin workspace. If you would like to change the name of the package then respective changes must be done in CMakeLists.txt and package.xml files.
3) Make the controlHusky.cpp file executable by executing following commands in your terminal. (Pre-requisite: Make sure your ROS environment is running)
  -> cd ~/catkin_ws
  -> cd src/deepfield_task/src/
  -> chmod +x controlHusky.cpp
4) Build the node
  -> cd ~/catkin_ws
  -> catkin_make
  -> source devel/setup.bash

# How to run the node?
Make sure you have turned on the ROS environment.
1) In one of the terminals execute husky_gazebo 
  -> cd ~/catkin_ws
  -> roslaunch husky_gazebo husky_empty_world.launch
2) In another terminal, you can now launch the node by following command:
  -> cd ~/catkin_ws
  -> rosrun deepfield_task controlHusky _x:=<input_x_goal_coordinate> _y:=<input_y_goal_coordinate>
    Example: rosrun deepfield_task controlHusky _x:=2 _y:=3

