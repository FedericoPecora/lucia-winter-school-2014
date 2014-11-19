===============
Pre-requisites: 
===============
Before installing the software, please install the following dependencies:
> sudo apt-get install ros-hydro-turtlebot 
> sudo apt-get install ros-hydro-turtlebot-simulator 
> sudo apt-get install ros-hydro-turtlebot-gazebo 
> sudo apt-get install ros-hydro-turtlebot-navigation 


===============================================
Installing the Lucia simulator and robot nodes: 
===============================================

1- Copy kobuki_gazebo_plugins package to your <ros_workspace>/src

2- Build kobuki_gazebo_plugins:
> cd <ros_workspace>
> catkin_make gazebo_ros_kobuki 

3- Its good to make a backup of your libgazebo_ros_kobuki.so 
> sudo mv /opt/ros/hydro/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so.backup 

3- Add newly compiled kobuki_gazebo_plugin to ROS libs:
> sudo mv ~/ros_workspace/devel/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so 

4- Copy lucia_sim_2014 to your <ros_workspace>/src and build it:
> cd <ros_workspace>
> catkin_make lucia_sim_2014_generate_messages
> catkin_make lucia_sim_2014

5- Launch the simulator:
> roscd lucia_sim_2014/launch 
> roslaunch environment.launch 

(the Gazebo environment will appear with four robots and four panels and few obstacles)

6- Launch rviz: 
> roscd lucia_sim_2014/launch/rviz 
> roslaunch view_navigation.launch 

There are four "2D Nav Goal" buttons in the rviz interface, one for each robot. To test navigation, click one of the "2D Nav Goal" buttons, then and click on the map where you want the TurtleBot to drive and drag to set the theta of the goal pose.


============================
Steps to launch the example: 
============================

1- Start the simulator and rviz as done above (steps 5 and 6)

2- Launch the example node
> roscd lucia_sim_2014/launch 
> roslaunch example.launch 

The example node dispatches each of the four robots to a panel. If a robot does not detect panel when it reaches its destiation, it performs a 360 deg rotation, which stops early if a panel is detected.

Panels are associated to integers, and the robots provide a service (see below) to query which panel (if any) is detected, the return value of which is the integer code associated to the panel (or -1 for no panel).

In the simulator, robots detect panels based on color. The following is the correspondence of colors to panels:

Black: 0
Red:   1
Blue:  2
Green: 3
None: -1

In the real deployment, panel detection is done via QR code recognition. Each QR code encodes the integer associated its panel.


====================
Service description: 
====================

In the following, "robot_name" stands for one of "turtlebot_n", where n = {1..4}. Each robot runs three services: 

1- /robot_name/getLocation : returns the current location of robot in the form of (x,y,theta)

Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/getLocation "read: 0"

2- /robot_name/getQR : returns an integer corresponding to the currently seen panel

Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/getQR "read: 0"

3- /robot_name/sendGoal : requests the robot to reach a position in map expressed in the form (x,y,theta)

Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/sendGoal "x: 0.0 y: 0.0 theta: 0.0"
 
A node called panel_serv provides the following global service:

/getPanel : provides a list of panels and their locations (a pair of points (x,y) for each panel)

Note: you can try this service by issuing the following on the command line:
> rosservice call /getPanel "read: 0"
