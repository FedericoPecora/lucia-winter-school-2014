#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <lucia_sim_2014/getQR.h>
#include <lucia_sim_2014/sendGoal.h>
#include <lucia_sim_2014/getLocation.h>
#include <lucia_sim_2014/getPanel.h>

#define FREQUENCY	10
#define READ 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
nav_msgs::OccupancyGrid map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

