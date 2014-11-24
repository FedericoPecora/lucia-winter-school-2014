/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2013, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors: Ali Abdul Khaliq on 12/11/2014
*********************************************************************/

#include "lucia_services.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{

  ros::init(argc, argv, "lucia_services");

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;

  image_sub_                    = it_.subscribe("camera/rgb/image_raw", 1, &imageCb);
  ros::ServiceServer Statusserv = nh_.advertiseService("getStatus", sendStatus);
  ros::ServiceServer QRserv     = nh_.advertiseService("getQR", sendQR);
  ros::ServiceServer Goalserv   = nh_.advertiseService("sendGoal", sendGoal);
  ros::ServiceServer Locserv    = nh_.advertiseService("getLocation", getLocation);
  ros::Subscriber    amcl_sub   = nh_.subscribe("amcl_pose", 10, amclCallback);
  ros::Subscriber sub = nh_.subscribe("move_base/status", 10, goalStatus);
  ros::Publisher rotate_pub = nh_.advertise<geometry_msgs::Twist>("commands/velocity", 100);

  ros::Rate loop_rate(FREQUENCY);

   while (ros::ok())
   {
    rotate(nh_,rotate_pub);
    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;
}

//==================================================
void imageCb(const sensor_msgs::ImageConstPtr& msg)
//==================================================
 {
  cv_bridge::CvImagePtr cv_ptr;
  try
   {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
  catch (cv_bridge::Exception& e)
   {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
   }

  cv::Vec3b intensity;
  int red=0;
  int green=0;
  int blue=0;
  int black=0;

  for(int i=0;i<(cv_ptr->image.rows);i++)
     {
     for(int j=0;j<(cv_ptr->image.cols);j++)
       {
   intensity = cv_ptr->image.at<cv::Vec3b>(i, j);
   if(intensity.val[0]>PIX_MAX && intensity.val[1]<PIX_MIN && intensity.val[2]<PIX_MIN) {red++;  }
   if(intensity.val[0]<PIX_MIN && intensity.val[1]>PIX_MAX && intensity.val[2]<PIX_MIN) {green++;}
   if(intensity.val[0]<PIX_MIN && intensity.val[1]<PIX_MIN && intensity.val[2]>PIX_MAX) {blue++; }
   if(intensity.val[0]<PIX_MIN && intensity.val[1]<PIX_MIN && intensity.val[2]<PIX_MIN) {black++;}
       }
      }

  code=-1;

  if(red   > PIX_THRESHOLD) {code=1;}
  if(green > PIX_THRESHOLD) {code=2;}
  if(blue  > PIX_THRESHOLD) {code=3;}
  if(black > PIX_THRESHOLD) {code=0;}

 red=0;
 green=0;
 blue=0;
 black=0;
 black=0;

  }

//========================================================================
bool sendQR(lucia_sim_2014::getQR::Request &req, lucia_sim_2014::getQR::Response &res)
//========================================================================
 {
  res.qrcode= code;
  return true;
 }

//========================================================================
bool sendStatus(lucia_sim_2014::getStatus::Request &req, lucia_sim_2014::getStatus::Response &res)
//========================================================================
 {
  res.status = statusOfMove;
  return true;
 }

//================================================================================
bool sendGoal(lucia_sim_2014::sendGoal::Request &req, lucia_sim_2014::sendGoal::Response &res)
//================================================================================
 {
  MoveBaseClient ac("move_base", true);

  statusOfMove = 1;
  while (!ac.waitForServer(ros::Duration(2.0)))
    {
    ROS_INFO("Waiting for the move_base action server to come up");
    }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id  = "/map"; 
  goal.target_pose.header.stamp =  ros::Time::now();
  geometry_msgs::Point goalPoint;

  goalPoint.x = req.x;
  goalPoint.y = req.y;
  goal.target_pose.pose.position = goalPoint;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(req.theta);//orientationPoint;

  ac.sendGoal(goal);

  res.result = 1;
  return true;
 }

//=========================================================================================
bool getLocation(lucia_sim_2014::getLocation::Request &req, lucia_sim_2014::getLocation::Response &res)
//=========================================================================================
 {
  btScalar roll, pitch, yaw;
  btQuaternion q(amcl_pos.pose.pose.orientation.x,
                 amcl_pos.pose.pose.orientation.y,
                 amcl_pos.pose.pose.orientation.z,
                 amcl_pos.pose.pose.orientation.w);

  btMatrix3x3(q).getEulerYPR(yaw, pitch,roll );

  res.id = ROBOT_ID;
  res.x= amcl_pos.pose.pose.position.x;
  res.y= amcl_pos.pose.pose.position.y;
  res.z= amcl_pos.pose.pose.position.z;
  res.theta = yaw;

  return true;
 }

//====================================================================
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
//====================================================================
 {
  amcl_pos.pose.pose.position.x = msg.pose.pose.position.x;
  amcl_pos.pose.pose.position.y = msg.pose.pose.position.y;
  amcl_pos.pose.pose.position.z = msg.pose.pose.position.z;

  amcl_pos.pose.pose.orientation.x= msg.pose.pose.orientation.x;
  amcl_pos.pose.pose.orientation.y= msg.pose.pose.orientation.y;
  amcl_pos.pose.pose.orientation.z= msg.pose.pose.orientation.z;
  amcl_pos.pose.pose.orientation.w= msg.pose.pose.orientation.w;
 }

//==================================================
void goalStatus(const actionlib_msgs::GoalStatusArray& msg)
//==================================================
 {
  status.status_list= msg.status_list; 
 }

//==================================================
void rotate(ros::NodeHandle nh_, ros::Publisher rotate_pub)
//==================================================
 {
  geometry_msgs::Twist pose;

  pose.linear.x = 0.0;
  pose.linear.y = 0.0;
  pose.linear.z = 0.0;
  pose.angular.x = 0.0;
  pose.angular.y = 0.0;
  pose.angular.z = 0.5;

  btScalar roll, pitch, yaw;

  btQuaternion q(amcl_pos.pose.pose.orientation.x,
                 amcl_pos.pose.pose.orientation.y,
                 amcl_pos.pose.pose.orientation.z,
                 amcl_pos.pose.pose.orientation.w);

  btMatrix3x3(q).getEulerYPR(yaw, pitch,roll );

  if(!status.status_list.empty()           &&
     (int)status.status_list[0].status==SUCCEEDED  &&
     code<0 && curr_yaw<=(2*M_PI))
     {
     if(init)
       {
       last_yaw=yaw;
       init=false;
       }
     else
       {
       curr_yaw+= abs(abs(yaw)- abs(last_yaw)); 
       last_yaw = yaw;
       }
    rotate_pub.publish(pose);
     }

  //Set status to report if asked thru getStatus service
  if (status.status_list.empty()) {
    statusOfMove = -1;
    }
  else if (!status.status_list.empty() && (int)status.status_list[0].status==SUCCEEDED  && code<0 && curr_yaw<=(2*M_PI)) {
    statusOfMove = 1;
  }
  else if (!status.status_list.empty() && (int)status.status_list[0].status!=ACTIVE) {
    statusOfMove = -1;
    }
  else if (!status.status_list.empty() && (int)status.status_list[0].status==ACTIVE) {
    statusOfMove = 1;
    }


  /*  
  else if (!status.status_list.empty() && (int)status.status_list[0].status!=ACTIVE) {
    statusOfMove = -1;
    }
  */

  if(!status.status_list.empty() &&
      (int)status.status_list[0].status==ACTIVE)
    {
    last_yaw=0;
    curr_yaw=0;
    init=true;
    } 
}

//======================================================================================//
//					EOF						//
//======================================================================================//
