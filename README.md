All files came from:  
https://navigation.ros.org/setup_guides/odom/setup_odom.html  
The purpose of this repository is to write down notes while learning ROS2.  

Required ROS2 packages
- robot-state-publisher
- joint-state-publisher
- xacro

This is a continuation from the repository display_urdf_rviz

# Background
The ultimate goal of this project is to set up a robot in Gazebo simulation and publish its odometry information. The differential drive and imu plugins are used to simulate the wheel encoder and imu sensor, which are commonly used to calculate odometry. The measurements from these two sensors are fed into the robot_localization package to produce a smoothed odometry information.

Note: RViz uses urdf files and Gazebo uses sdf files to describe a robot model. However, urdf files can be used for Gazebo, which is what is done in this tutorial.  

# Explanation of the plugins and packges used
**Differential drive plugin:**  
This plugin simulates the odometry calculations for a two-wheeled robot. For a two-wheeled robot with wheel encoders, the odometry information (linear and angular velocity of the robot) would be calculated using the formula:  
![image](https://user-images.githubusercontent.com/102873080/235415384-30234f02-da1a-4eac-aa85-c2c84b34ef9c.png)  
The differential drive plugin does simulates this calculation by subscribing to the /demo/cmd_vel topic and publishing to the /demo/odom topic.
The plugin can be added by adding the following lines to the urdf file.  


**Imu plugin:**  
Imu sensors in real life measures the the velocity and acceleration of the robot using its accelerometer. The imu sensor plugin added to the robot simulates this by providing the 
There is noise added to the imu data to best imitate imu measurements in real life.  

**joint-state-publisher package:**  
The joint-state-publisher publishes the state (position and velocity) of all the **NON-FIXED** joints in a robot model provided by a urdf file. The information is published to the /joint_state topic.  
This is the data published on the /joint_states topic for the robot model in this tutorial. Only the states of drivewhl_l_joint and drivewhl_r_joint are published because they are the only joints that aren't fixed (they are revolute joints). The position data shows the angular orientation of the joints because they are revolute joints and it matches the joint-state-publisher-gui.

**robot-state-publisher package:**  
The robot-state-publisher listens to the /joint_states topic published to by the joint-state-publisher package and calculates the transforms between all the joints in the robot. The transform data is then published to the /tf topic.  






