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

While RViz uses urdf files to describe the robot, Gazebo uses sdf files, which has a very similar xml-type syntax. Both urdf and sdf files can and are usually exported from CAD models, but simple models can be written by hand.

# Problems faced
I wrote the urdf file by hand referencing the urdf file in the website I cited above. Then I tried to generate an sdf file from the urdf file using the terminal command ***gz sdf -p <urdf_file_name> <sdf_file_name>***. The robot model from the sdf file on Gazebo visually looked the same as the robot model on RViz. When I tried to move the robot using ***rqt_robot_steering*** however, it started to oscillate while moving forward. The problem seemed to be with the inertia of the robot. So, I removed all the inertia components from the urdf file and tried generating the sdf from the modified urdf file. However, terminal command just did not work this time as the inertia components are required to generate the sdf file. So, I just opted to use the sdf file provided in the website (which does not have inertia components, so I don't even know how this sdf file works).  

The robot model in Gazebo is as shown.  
![image](https://user-images.githubusercontent.com/102873080/235328756-1f8783bd-5a21-4ce2-b089-d69acdcf34dc.png)  
It is a two-wheeled rohttps://github.com/Juhyung-L/display_sdf_gazebo_tutorial/blob/f31f031fd015f6e295ac1d3acf6e5a3206020ab2/models/mobile_bot_model/model.sdf#L185-L228bot with one caster wheel at the front.

# Plugins and packges used
**Differential drive plugin:**  
To simulate the robot moving with its two wheels, the differential drive plugin was used.  
The plugin is added by adding these lines to the sdf file.
https://github.com/Juhyung-L/display_sdf_gazebo_tutorial/blob/f31f031fd015f6e295ac1d3acf6e5a3206020ab2/models/mobile_bot_model/model.sdf#L185-L228
The plugin subscribes to the /cmd_vel topic, which has velocity information published by rqt_robot_steering, and publishes odometry information to the /wheel/odometry topic and it also moves the robot on Gazebo.  

**Imu plugin:**  
This plugin is added to simulate an imu attached to the robot. It will be used in later tutorials along with wheel odometry information to get a smoothed odometry information of the robot.  
The plugin is added by adding these lines to the sdf file.
https://github.com/Juhyung-L/display_sdf_gazebo_tutorial/blob/f31f031fd015f6e295ac1d3acf6e5a3206020ab2/models/mobile_bot_model/model.sdf#L105-L183  
There is noise added to the imu data to best imitate imu measurements in real life.  

**joint-state-publisher package:**  
The joint-state-publisher publishes the state (position and velocity) of all the **NON-FIXED** joints in a robot model provided by a urdf file. The information is published to the /joint_state topic.  
![image](https://user-images.githubusercontent.com/102873080/235330499-9e39e2a7-e833-4c32-bddf-395b2d3e906a.png)
This is the data published on the /joint_states topic for the robot model in this tutorial. Only the states of drivewhl_l_joint and drivewhl_r_joint are published because they are the only joints that aren't fixed (they are revolute joints). The position data shows the angular orientation of the joints because they are revolute joints and it matches the joint-state-publisher-gui.

**robot-state-publisher package:**  
The robot-state-publisher listens to the /joint_states topic published to by the joint-state-publisher package and calculates the transforms between all the joints in the robot. The transform data is then published to the /tf topic.  
![image](https://user-images.githubusercontent.com/102873080/235330772-83d7e55b-7831-4fff-985d-fe7a444ec1b3.png)  






