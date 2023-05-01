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
The exact message published on the /demo/odom topic is:  
![image](https://user-images.githubusercontent.com/102873080/235417061-fcad4831-3ad2-43b5-9883-e5cb1c092cd2.png)  
The plugin can be added by adding the following lines to the urdf file.  
<add code>
Note that /<publish_wheel_tf> is set to false. The code snippet provided by the official Nav2 tutorial has this set to true. This will publish the base_link to wheel transform to the tf2 topic. However, the robot state publisher already provides that information. So, setting the option to true will have two nodes (differential drive plugin and robot_state_publisher package) publishing the same information to the /tf topic, which will result in an warning saying that the data from one of the two sources will be ignored.  

**Imu plugin:**  
Imu sensors in real life measures the the linear and angular velocity and acceleration of the robot using its accelerometer and gyroscope. The imu sensor plugin added to the robot simulates this by providing the same information on the /demo/imu topic.  
The exact message published on the /demo/imu topic:  
![image](https://user-images.githubusercontent.com/102873080/235417222-ef8c4b88-e751-457c-be6f-d6da89a8315f.png)  
- It provides orientation, angular velocity, and linear acceleration.
- There is noise added to the data to best imitate imu measurements in real life.

The plugin can be added by adding the following lines to the urdf file.
<add code>

**joint-state-publisher package:**  
The joint-state-publisher publishes the state (position and velocity) of all the **NON-FIXED** joints in a robot model provided by a urdf file. The information is published to the /joint_state topic.  
![image](https://user-images.githubusercontent.com/102873080/235418386-10c632fd-c202-4735-9243-31e073273610.png)  
This is the data published on the /joint_states topic for the robot model in this tutorial. Only the states of drivewhl_l_joint and drivewhl_r_joint are published because they are the only joints that aren't fixed (they are revolute joints). The position data shows the angular orientation of the joints because they are revolute joints and it matches the joint-state-publisher-gui.

**robot-state-publisher package:**  
The robot-state-publisher listens to the /joint_states topic published to by the joint-state-publisher and the robot model description in the urdf file and calculates the transforms between all the joints in the robot. The transform data is then published to the /tf topic.  
This is the transform data for transform from base_link to drivewhl_l_link.  
![image](https://user-images.githubusercontent.com/102873080/235418943-5c8f095a-5a96-4e31-8d22-7e194483838c.png)  

# Robot localization package
In real life, calculating the odometry of the robot from a single sensor type is not ideal. For example, calculating the odometry from the wheel encoders only could lead to inaccurate information overtime due to wheel slip. Consider the following scenario: the robot bumps into a wall and the wheels lose contact from the ground for a brief period of time. If the wheels are spinning while in the air, the odometry from the encoders will say that the robot moved while in reality it did not. Because of this, odometry is usually calculated by fusing the data from two or more different types of sensors. The fusion is usually done thorugh a variation of the Kalman Filter, which is what the robot localization package uses. The package takes in odometry information from two different types of sensors and uses the Extended Kalman Filter (ekf) to calculate a more reliable odometry information. In this project, the odometry information from /demo/odom and /demo/imu was fused to publish /accel/filtered and /odometry/filtered.  

The following yaml file is added to the config file to provide the specifications.  
<add code>
There is a matrix of booleans associated with both /demo/odom and /demo/imu. This matrix represents which data from the two sources of odometry the robot localization package will use.  
The matrix represents:  
[x,&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;y,&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;z,  
&nbsp;roll,&nbsp;&nbsp;&nbsp;pitch,&nbsp;&nbsp;yaw,  
&nbsp;vx,&nbsp;&nbsp;&nbsp;&nbsp;vy,&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;vz,  
&nbsp;vroll,&nbsp;vpitch,&nbsp;vyaw,  
&nbsp;ax,&nbsp;&nbsp;&nbsp;&nbsp;ay,&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;az]  

So, the following data is being put into the filter  
For /demo/odom
- x, y, z, vyaw

For /demo/imu
- roll, pitch, yaw  

Choosing which information to feed into the robot localization package requires information about the sensor readings. Ideally, you would fuse the best/least noisy measurements from each sensor to produce the best odometry. The official Nav2 tutorial suggests not fusing data that are derivatives of each other.

# final result
Running rqt_graph should yielded the following graph  
![image](https://user-images.githubusercontent.com/102873080/235422210-7bad34ce-967c-4ff5-ad52-9c2abace92ab.png)  

Data published on /odometry/filtered topic:  
![image](https://user-images.githubusercontent.com/102873080/235422571-793cd521-4f11-4b8c-bf9a-1eb99b718c1f.png)  

Data published on /accel/filtered topic:  
![image](https://user-images.githubusercontent.com/102873080/235422480-22c3f82f-ffa5-455a-b078-c24c8a1f250a.png)  










