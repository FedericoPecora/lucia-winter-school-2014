# Codebase for Lucia Winter School 2014 (Lucia simulator, Services and Meta-CSP Lecture) #

This project contains sources for the Lucia simulation infrastructure and services (used during the "Constraint-Based Reasoning for Robots" lecture on Tuesday, and the ASP lecture on Wednesday).  It also contains the ROS/Java codebase for the "Constraint-Based Reasoning for Robots" lecture.

Four packages are provided:

  * `kobuki_gazebo_plugins`: provides simulated Turtlebot 2 for use with the Gazebo simulator - this is a customized version of the default package with added support for multiple robots.

  * `launch_services`: contains launch files for starting the robots and services used in the lectures.

  * `lucia_meta_csp_lecture`: the ROS/Java codebase for the "Constraint-Based Reasoning for Robots" lecture.

  * `lucia_sim_2014`: provides a simulated environment (used by Gazebo) and the implementations of the services provided by the simulated setup.

  * `services`: service definitions (`srv` files).

## Pre-requisites ##
You should have already followed the ROS installation guides provided on the school website.  In short, this means that you have:

  * installed ROS Hydro ([tutorial](http://wiki.ros.org/ROS/Tutorials))

  * installed the Java Development Kit (`sudo apt-get install openjdk-7-jdk`)

  * installed ROSJava (http://wiki.ros.org/rosjava)

  * installed the Eclipse IDE (https://www.eclipse.org/downloads/)

Henceforth we assume that your ROS workspace directory is `<CATKIN_WS>`, and `<robot>` stands for one of `turtlebot_n`, where n = {1..4}.

## Installation of provided packages ##

Before installing the provided packages, please install the following dependencies:

```
$> sudo apt-get install ros-hydro-turtlebot 
$> sudo apt-get install ros-hydro-turtlebot-simulator 
$> sudo apt-get install ros-hydro-turtlebot-gazebo 
$> sudo apt-get install ros-hydro-turtlebot-navigation
```

Check out the latest version of the simulator from Google Code into the source sub-directory of your ROS workspace:

```
$> cd <CATKIN_WS>/src
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/kobuki_gazebo_plugins
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/launch_services
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/lucia_meta_csp_lecture
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/lucia_sim_2014
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/lucia_simulator/services
$> svn checkout http://lucia-winter-school-2014.googlecode.com/svn/trunk/asp_session_files
```

Build everything:

```
$> cd ..
$> catkin_make
```

If you get an error, just do:

```
$> catkin_make services_generate_messages
$> catkin_make
```

In order to simulate multiple robots, you need to replace the existing Turtlebot 2 simulation implementation with the one we have provided.  First, make a backup of your `libgazebo_ros_kobuki.so`:

```
$> sudo mv /opt/ros/hydro/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so.backup 
```

Then add newly compiled kobuki\_gazebo\_plugin to ROS libs:

```
$> sudo mv devel/lib/libgazebo_ros_kobuki.so /opt/ros/hydro/lib/libgazebo_ros_kobuki.so 
```


## Testing the Simulator ##

You can now test the simulator:

```
$> roslaunch lucia_sim_2014 environment.launch 
```

(the Gazebo environment will appear with four robots and six panels.)

You can also launch `rviz` to see what the robots see and post goals to them:

```
$> roslaunch lucia_sim_2014 view_navigation.launch 
```

There are four "2D Nav Goal" buttons in the rviz interface, one for each robot. To test navigation, click one of the "2D Nav Goal" buttons, then and click on the map where you want the TurtleBot to drive and drag to set the theta of the goal pose.

Finally, you can joystick a robot around by launching the following:

```
$> roslaunch launch_services joystick_turtlebot.launch robot:=<robot>
```

This will work both in simulation and with the real robots.

## Testing code for Constraint-Based Reasoning lecture ##

To launch the "final product" you will obtain at the end of the lecture, do the following:

```
$> roslaunch lucia_meta_csp_lecture cbr_lecture.launch
```

This starts Gazebo and all other services needed.  When you see a window with timelines (as many as there are robots), you can tell the robots to start sensing, which is done via publishing the following topic:

```
$> rostopic pub active_sensing std_msgs/String "active"
```

You can kill this topic when you see the robots start moving.

## Setting up Eclipse for exercise development ##

The ROSJava package containing the exercises is called `active_perception`.  You can use Gradle to prepare an Ecipse project, hence getting all the necessary dependencies configured automatically (or almost, see below).  To do this, enter the following:

```
$> cd <CATKIN_WS>/src/lucia_meta_csp_lecture
$> ./gradlew eclipse
```

Now open Eclipse, and do the following:

  * Create a new Java project, un-click "use default location", navigate to `<CATKIN_WS>/src/lucia_meta_csp_lecture/active_perception`, click finish.

There is one "automatic" configuration you have to fix:

  * Right click on the newly created project `active_perception`, select "properties", select "Java Build Path" on the left side navigation tree;

  * Select the "Projects" tab, remove the "services" project;

  * Select the "Library" tab, click "Add external jars", navigate to `<CATKIN_WS>/src/lucia_meta_csp_lecture/services/build/libs`, select `services-0.0.0.jar`, and finally click "Ok".

Now refresh the `active_perception` project, and yuo are ready to go.

## Running with the real robots ##

There is a "ROS Master" laptop near the arena, with IP address 10.0.0.22.  Username is `fpa` and password is `nordiclight`.  From the master machine, ssh into the robot(s) you will use (the password is `turtlebot`):

```
$> ssh <robot>
```

On each robot, launch the services:

```
$> roslaunch lucia_services lucia_turtlebot.launch robot_name:=/<robot>
```

Note: It will take few seconds to load everything and finally the message "odom received!" will appear.

On the ROS master machine, launch rviz and estimate the initial pose of the robots:

```
$> roslaunch launch_services view_navigation.launch
```

Connect your machine to the wired LAN next to the ROS master machine. You will get an IP address - note it down. Now open your `~/.bashrc` file and add the following lines:

```
export ROS_MASTER_URI=http://10.0.0.22:11311
export ROS_HOSTNAME=<YOUR_IP>
```

You are now ready to start your nodes - enjoy!

## Description of provided services ##

Each robot runs three services:

  * `/<robot>/getLocation`: returns the current location of robot in the form of (x,y,theta).

You can try this service by issuing the following on the command line:

```
$> rosservice call /turtlebot_1/getLocation "read: 0"
```

  * `/<robot>/getQR`: returns an integer corresponding to the currently seen panel. In the simulator, robots detect panels based on color, while in the real deployment panel detection is done via QR code recognition. The following is the correspondence of colors to panels:

```
(Red:1, Green:2, Blue:3, Black:4, Purple:5, Yellow:6, None: -1)
```

You can try this service by issuing the following on the command line:

```
$> rosservice call /turtlebot_1/getQR "read: 0"
```

  * `/<robot>/sendGoal`: requests the robot to reach a position in map expressed in the form (x,y,theta,rotationAfter). Set rotationAfter to 1 if a rotation should occur after the robot's initial movement. The rotation is terminated either after 360 degrees have been swept, or when a QR code is seen. If this behavior is not required, set rotationAfter to 0.

You can try this service by issuing the following on the command line:

```
$> rosservice call /turtlebot_1/sendGoal "x: 0.0 y: 0.0 theta: 0.0 rotationAfter: 1"
```

  * `/<robot>/rotate`: requests the robot to rotate 360 degrees to search for a QR code.

You can try this service by issuing the following on the command line:

```
$> rosservice call /turtlebot_1/rotate "rotate: 1"
```

  * `/getPanel`: provides a list of panels and their locations (a pair of points (x,y) for each panel).

You can try this service by issuing the following on the command line:

```
$> rosservice call /getPanel "read: 0"
```