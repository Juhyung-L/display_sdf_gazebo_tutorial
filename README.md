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
This plugin simulates the encoders on the two wheels of the robot. It takes in geometry_msgs?Twist published on the /cmd_vel topic and publishes nav_msgs/Odometry on the /odom topic. So basically, it converts the robot's velocity into wheel encoder readings.

**Imu plugin:**  
This plugin is added to simulate an imu attached to the robot. It will be used in later tutorials along with wheel odometry information to get a smoothed odometry information of the robot.  
The plugin is added by adding these lines to the sdf file.

There is noise added to the imu data to best imitate imu measurements in real life.  

**joint-state-publisher package:**  
The joint-state-publisher publishes the state (position and velocity) of all the **NON-FIXED** joints in a robot model provided by a urdf file. The information is published to the /joint_state topic.  
This is the data published on the /joint_states topic for the robot model in this tutorial. Only the states of drivewhl_l_joint and drivewhl_r_joint are published because they are the only joints that aren't fixed (they are revolute joints). The position data shows the angular orientation of the joints because they are revolute joints and it matches the joint-state-publisher-gui.

**robot-state-publisher package:**  
The robot-state-publisher listens to the /joint_states topic published to by the joint-state-publisher package and calculates the transforms between all the joints in the robot. The transform data is then published to the /tf topic.  






