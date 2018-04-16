#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "hl_controller/hl_controller.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "hl_controller/hl_helper.h"

hl_controller::hl_controller()
{
  tag_detection_sub = n.subscribe("/tag_detections", 1, &hl_controller::tag_detections_Callback, this);
  ground_truth_sub = n.subscribe("/ground_truth_to_tf/pose", 1, &hl_controller::ground_truth_Callback, this);
  mode_change_sub = n.subscribe("/hl_change_mode", 1, &hl_controller::mode_change_Callback, this);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/command/pose", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  client = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

  mode = "takeoff";

  init_motors();
  //enable_motors();
  
  ros::Rate loop_rate(100);
  int count = 0;
  while(ros::ok())
  {
      executeCB();   

      ros::spinOnce();
      loop_rate.sleep();
  }
}

hl_controller::~hl_controller()
{
}

//YT before init motors we need some time to wait for other ROS module
void hl_controller::init_motors()
{
  ros::Time t0 = ros::Time::now();
  while(1)
  {
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    if(d.toSec() > 1)
      break;
  }
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  client.call(srv);
}

void hl_controller::tag_detections_Callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    std::cout << "YT: find at least one target!" << std::endl;
    current_pose_in_target_frame.header.stamp = msg->detections.at(0).pose.header.stamp;
    current_pose_in_target_frame.header.frame_id = msg->detections.at(0).pose.header.frame_id;
    current_pose_in_target_frame.pose.position.x = msg->detections.at(0).pose.pose.pose.position.x;
    current_pose_in_target_frame.pose.position.y = msg->detections.at(0).pose.pose.pose.position.y;
    current_pose_in_target_frame.pose.position.z = msg->detections.at(0).pose.pose.pose.position.z;
    current_pose_in_target_frame.pose.orientation.x = msg->detections.at(0).pose.pose.pose.orientation.x;
    current_pose_in_target_frame.pose.orientation.y = msg->detections.at(0).pose.pose.pose.orientation.y;
    current_pose_in_target_frame.pose.orientation.z = msg->detections.at(0).pose.pose.pose.orientation.z;
    current_pose_in_target_frame.pose.orientation.w = msg->detections.at(0).pose.pose.pose.orientation.w;
 
}

void hl_controller::ground_truth_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    current_pose_in_world_frame.
}

void hl_controller::mode_change_Callback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "YT: receive new flight mode" << std::endl;
    mode = "receiveyt";
}

void hl_controller::executeCB()
{
   if(mode == "takeoff")
   {
      geometry_msgs::PoseStamped pose_sp;
      pose_sp.pose.position.x = 0.0;
      pose_sp.pose.position.y = 0.0;
      pose_sp.pose.position.z = 10.0;
      
      std::cout << "hl666" << std::endl;
      pose_pub.publish(pose_sp);
   }
   else if(mode == "receiveyt")
   {
      geometry_msgs::PoseStamped pose_sp;
      pose_sp.pose.position.x = 0.0;
      pose_sp.pose.position.y = 0.0;
      pose_sp.pose.position.z = 1.0;
      
      std::cout << "hl666" << std::endl;
      pose_pub.publish(pose_sp);
   }
}

