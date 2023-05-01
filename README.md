All files in this repository came from https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  
The purpose of this repository is to write down notes while learning ROS2.

Required ROS2 packages
- robot-state-publisher
- joint-state-publisher
- joint-state-publisher-gui
- xacro

# Background
A .urdf file represents a robot model using joints and links.
- Links: the rigid skeleton of the robot
- Joints: joins the links together

# Creating a ROS2 package
Make a folder and a folder named source in it.  
***mkdir ~/dev_ws/src***  
***cd ~/dev_ws/src***  
***ros2 pkg create --build-type ament_cmake mobile_bot***  
- This will make a folder named mobile_robot inside the src folder
- This folder will have
  - CMakeLists.txt
  - package.xml
  - include folder
  - src folder

Then make launch, models, and rviz folders inside the package.  
***cd ~/dev_ws/src/mobile_bot***  
***mkdir launch models rviz***  
- launch folder will have the launch script in Python
- rviz folder will have the rviz launch file
- models folder will have the .urdf file of the robot model

Go to the root directory of the package and build.  
***cd ~/dev_ws***  
***colcon build***  

# Make the .urdf file
Go to the models folder and make a mobile_bot_model.urdf file  
***cd ~/dev_ws/src/mobile_bot/models***  
***gedit mobile_bot_model.urdf***  

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L4-L15
- Using xacro to define constants that will be reused throughout the file

This is the format to define a link.
- Start with \<link name="some_name"\> and end with \</link\> (on the same tab spacing)
- \<link\> has the components \<visual\>, \<in<uri>model://mobile_bot_model_sdf</uri>ertia\>, and \<collision\>
- \<visual\> has components \<geometry\> (for defining shape) and \<material\> (for defining color)

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L41-L50
- Defining the rectangular body of the robot

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L59-L60
- Defining the virtual link (doesn't exist in real life) that is directly under the center of the robot's body  
- Since it is a virtual link, it has no components

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L62-L66
- Defining the joint that connects the links robot_base and base_link
- The joint type is "fixed" because robot_base and base_link do not move relative to each other (you can use static coordinate transform between them)

***So basically, a .urdf file is just defining links and connecting them with joints***  

The \<visual\> component is all you need to display the robot model on RViz because RViz is just a visualization tool. But if you want the robot to be simulated on Gazebo, you also need the \<inertia\> and \<collision\> components because Gazebo has a physics enigne (like a physics engine for a game) 

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L17-L39
- Defining the inertia of the polygons that make up the robot model using known equations
- xacro is used like a function where the function name is "box_inertia" and the input parameters are "m w h d" (mass width, height, depth?)

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L56
- xacro is used to define the inertia of the box-shaped body of the robot
- It essentially copies and pastes the xacro defined above but with the input parameters defined

https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/953928196113ce88a06cc991bceaef9e26573014/models/mobile_bot_model.urdf#L51-L55
- defining the collision box (almost like a hitbox) for the robot's body

Also add the .rviz file into the rviz folder.  
I don't know how you generate this file.  

# Add dependencies
Add these lines to the package.xml file.
https://github.com/Juhyung-L/display_urdf_rviz_tutorial/blob/4124760ed5a443e342c3dd328c788c80ef7e4a41/package.xml#L12-L16

# Make the launch file
***cd ~/dev_ws/src/mobile_bot/launch***  
***gedit display.launch.py***  
- launch files in Python end with .launch.py

The launch file is just a Python script to automatically set all the configurations and launch RViz

# Building the package
Add these lines to the CMakeLists.txt.  
***cd ~/dev_ws***  
***colcon build***  

After the building, add these lines to the ~/.bashrc file.  
***source ~/dev_ws/install/setup.bash***  
- this will source the package everytime the terminal is opened
- sourcing sets the appropriate environment variables

# Displaying the robot in RViz
Open a new terminal and type:  
***ros2 launch mobile_bot display.launch.py***  
This will bring up the RViz window and the joint_state_publisher_gui. 
![image](https://user-images.githubusercontent.com/102873080/233770287-5e14b63d-02de-48c5-9315-f1f603e485d2.png)  

The joint_state_publisher_gui is used to change the angle of the motor.  

# Viewing the coordinate transform
***ros2 run tf2_ros tf2_echo base_link front_caster***  
Syntax is: ***ros2 run tf2_ros tf2_echo parent frame child frame***  
![image](https://user-images.githubusercontent.com/102873080/233770431-a1d8117b-3d7f-49ca-aa4a-136443f1ad8f.png)  
The "Translation" says that front_caster is located at point [0.14, 0, -0.09] in base_link's frame. The "Rotation", which is represented in a Quaternion (a way of numerically representing orientation), is all [0, 0, 0, 1], which means that base_link and front_caster have the same orientation. The transform from base_link to front_caster is a static transform because front_caster is fixed in the base_link's coordinate frame.  
![image](https://user-images.githubusercontent.com/102873080/233770572-93484ec5-6720-4796-983f-daa346dae7c4.png)  
The transformation makes sense looking at the the RViz model.


