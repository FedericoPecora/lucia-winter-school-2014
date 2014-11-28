#include <ros/ros.h>
#include <tf/message_filter.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bullet/LinearMath/btMatrix3x3.h>
#include <lucia_sim_2014/getQR.h>
#include <lucia_sim_2014/getStatus.h>
#include <lucia_sim_2014/sendGoal.h>
#include <lucia_sim_2014/getLocation.h>
#include <lucia_sim_2014/getPanel.h>

#define FREQUENCY	100
#define FAIL		-1
#define ROBOT_ID	 1
#define PIX_THRESHOLD    80000
#define PIX_MIN          10
#define PIX_MAX          100
#define SUCCEEDED        3
#define ACTIVE           1

 bool init = true;
 double curr_yaw =0;
 double last_yaw =0;

 int code;   // 1:red, 2:green, 3:blue, 0:black, -1:non

 double panel1_x1, panel1_x2, panel1_y1, panel1_y2,
        panel2_x1, panel2_x2, panel2_y1, panel2_y2,
        panel3_x1, panel3_x2, panel3_y1, panel3_y2,
        panel4_x1, panel4_x2, panel4_y1, panel4_y2;

//statusOfMove < 0 <=> not moving
int statusOfMove = -1;

 using namespace std;
 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 geometry_msgs::PoseWithCovarianceStamped amcl_pos;
 actionlib_msgs::GoalStatusArray status;


  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  bool sendQR(lucia_sim_2014::getQR::Request &req, lucia_sim_2014::getQR::Response &res);
  bool sendStatus(lucia_sim_2014::getStatus::Request &req, lucia_sim_2014::getStatus::Response &res);
  bool sendGoal(lucia_sim_2014::sendGoal::Request &req, lucia_sim_2014::sendGoal::Response &res);
  bool getLocation(lucia_sim_2014::getLocation::Request &req, lucia_sim_2014::getLocation::Response &res);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
 void goalStatus(const actionlib_msgs::GoalStatusArray& msg);
 void rotate(ros::NodeHandle nh_, ros::Publisher rotate_pub);
 bool getPanel(lucia_sim_2014::getPanel::Request &req, lucia_sim_2014::getPanel::Response &res);

//======================================================================================//
//					EOF						//
//======================================================================================//

