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
0- Henceforth we assume that your ROS workspace directory is <catkin_ws>. And <robot> stands for one of "turtlebot_n", where n = {1..4}.

1- Check out the latest version of the simulator from Google Code into the source subdirectory of your ROS workspace:
> cd <catkin_ws>/src
> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/kobuki_gazebo_plugins
> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/launch_services
> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/lucia_meta_csp_lecture
> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/lucia_sim_2014
> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/services

2- Build everything:
> cd ..
> catkin_make

If you get an error, just do:

> catkin_make services_generate_messages
> catkin_make

===============
Lucia simulator
===============

1- Make a backup of your libgazebo_ros_kobuki.so 
> sudo mv /opt/ros/hydro/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so.backup 

2- Add newly compiled kobuki_gazebo_plugin to ROS libs:
> sudo mv devel/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so 

3- Launch the simulator:
> roslaunch lucia_sim_2014 environment.launch 
(the Gazebo environment will appear with four robots and four panels and few obstacles)

4- Launch rviz: 
> roslaunch lucia_sim_2014 view_navigation.launch 

There are four "2D Nav Goal" buttons in the rviz interface, one for each robot. To test navigation, click one of the "2D Nav Goal" buttons, then and click on the map where you want the TurtleBot to drive and drag to set the theta of the goal pose.

===============
Real robots
===============

1- From master maching ssh into the robots:
> ssh <robot>
password = turtlebot

2- Launch the services 
>roslaunch lucia_services lucia_turtlebot.launch robot_name:=/<robot>

3- Repeat step 1 and 2 for all robots.
Note: It will take few seconds to load everything and finally the message "odom received!" will appear.

4- On you own machine, open ~/.bashrc file and add the following lines in it:
export ROS_MASTER_URI=http://10.0.0.22:11311
export ROS_HOSTNAME=<YOUR_IP>

5- Launch rviz
>roslaunch launch_services view_navigation.launch

====================
Services description: 
====================

Each robot runs three services: 

1- /<robot>/getLocation : returns the current location of robot in the form of (x,y,theta)
Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/getLocation "read: 0"

2- /<robot>/getQR : returns an integer corresponding to the currently seen panel
Note: In the simulator, robots detect panels based on color and in the real deployment, panel detection is done via QR code recognition. The following is the correspondence of colors to panels:
(Black:0, Red:1, Green:2, Blue:3, Purple:4, Yellow:5, None: -1)
and you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/getQR "read: 0"

3- /<robot>/sendGoal : requests the robot to reach a position in map expressed in the form (x,y,theta,rotationAfter). Set rotationAfter to 1 if searching the QR code is required after reaching to the goal otherwise set it to 0.
Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/sendGoal "x: 0.0 y: 0.0 theta: 0.0 rotationAfter: 1"

4- /<robot>/rotate : requests the robot to rotate 360 degrees to search the QR code.
Note: you can try this service by issuing the following on the command line:
> rosservice call /turtlebot_1/rotate "rotate: 1"
 
5- /getPanel : provides a list of panels and their locations (a pair of points (x,y) for each panel)
Note: you can try this service by issuing the following on the command line:
> rosservice call /getPanel "read: 0"
