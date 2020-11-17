# Controlling-Husky-Robot
This repository is created to maintain a code in C++ language to control the motion of Husky Robot using ROS (Robot Operating System)

# How to install the node?
1) Make sure you have "husky_simulator" installed on your ROS. Follow ROS tutorial for CPP at http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
2) Clone the main branch of this repository into the src directory of your catkin workspace.
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

