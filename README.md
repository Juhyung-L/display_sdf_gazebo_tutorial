All files came from:  
https://navigation.ros.org/setup_guides/odom/setup_odom.html  
https://automaticaddison.com/set-up-the-odometry-for-a-simulated-mobile-robot-in-ros-2/  
The purpose of this repository is to write down notes while learning ROS2.  

Required ROS2 packages
- robot-state-publisher
- joint-state-publisher
- xacro

This is a continuation from the repository display_urdf_rviz_tutorial

# Background
RViz is a visual simulator meaning it only can only show the model of the robot. On the other hand, Gazebo is a complete simulator that allows you to test the robot using its physics engine. Gazebo was made to allow roboticists to simulate the robot as close to reality as possible before building the robot in real life.

While RViz uses a .urdf file to describe the robot, Gazebo uses a .sdf file, which has a very similar xml-type syntax. Both .urdf and .sdf files can and are usually exported from CAD models, but simple models like the one used in this tutorial can be written by hand.  

In this tutorial, the robot model was written in the model.xacro file in urdf syntax using xacro then converted to a .urdf file.  
***xacro model.xacro > model.urdf***
- terminal command used turn a .xacro file into a .urdf file

https://github.com/Juhyung-L/display_sdf_gazebo_tutorial/blob/b60d758c623880025fd0ba449df96c680c92ab7c/models/mobile_bot_model_urdf/model.xacro#L42-L60
Basically, the lines above turn into the lines below.
https://github.com/Juhyung-L/display_sdf_gazebo_tutorial/blob/b60d758c623880025fd0ba449df96c680c92ab7c/models/mobile_bot_model_urdf/model.urdf#L7-L29
All the xacro macros are replaced with constants that they represent.  
This is done first so that the model.urdf file can be converted into a model.sdf file using another terminal command.  
***gz sdf -p model.urdf > model.sdf***  
This command does not work if the .urdf file contains xacro macro, which is why the conversion process is done
- model.xacro -> model.urdf -> model.sdf

write about 
- moving the sdf file a folder with model.config in it.
- small_town.world 
- joint-state-publisher and robot-state-publisher
- 


