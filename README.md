All files came from:  
https://navigation.ros.org/setup_guides/odom/setup_odom.html  
The purpose of this repository is to write down notes while learning ROS2.  

Required ROS2 packages
- robot-state-publisher
- joint-state-publisher
- xacro

This is a continuation from the repository display_urdf_rviz_tutorial

# Background
The ultimate goal of this tutorial is to 

Note: RViz uses urdf files and Gazebo uses sdf files to describe a robot model. However, urdf files can be used for Gazebo, which is what is done in this tutorial.  


# Plugins and packges used
**Differential drive plugin:**  
To simulate the robot moving with its two wheels, the differential drive plugin was used.  
The plugin is added by adding these lines to the sdf file.

The plugin subscribes to the /cmd_vel topic, which has velocity information published by rqt_robot_steering, and publishes odometry information to the /wheel/odometry topic and it also moves the robot on Gazebo.  

**Imu plugin:**  
This plugin is added to simulate an imu attached to the robot. It will be used in later tutorials along with wheel odometry information to get a smoothed odometry information of the robot.  
The plugin is added by adding these lines to the sdf file.

There is noise added to the imu data to best imitate imu measurements in real life.  

**joint-state-publisher package:**  
The joint-state-publisher publishes the state (position and velocity) of all the **NON-FIXED** joints in a robot model provided by a urdf file. The information is published to the /joint_state topic.  
This is the data published on the /joint_states topic for the robot model in this tutorial. Only the states of drivewhl_l_joint and drivewhl_r_joint are published because they are the only joints that aren't fixed (they are revolute joints). The position data shows the angular orientation of the joints because they are revolute joints and it matches the joint-state-publisher-gui.

**robot-state-publisher package:**  
The robot-state-publisher listens to the /joint_states topic published to by the joint-state-publisher package and calculates the transforms between all the joints in the robot. The transform data is then published to the /tf topic.  






