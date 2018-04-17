#ifndef _HL_CONTROLLER_
#define _HL_CONTROLLER_
#include "ros/ros.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "hl_controller/hl_constants.h"
#include "hl_controller/hl_helper.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

class hl_controller
{

public:
  hl_controller();
  ~hl_controller();
  void init_motors();

  
  ros::Subscriber tag_detection_sub;
  //YT get the robot pose (from ground_truth or pose_estimator)
  ros::Subscriber robot_pose_sub;
  ros::Subscriber mode_change_sub;
  ros::Publisher pose_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher path_pub;
  ros::ServiceClient client;
  void tag_detections_Callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);
  void robot_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void mode_change_Callback(const std_msgs::String::ConstPtr& msg);
  void executeCB();
  void setspeed(double vx, double vy, double vz);
  void set_velocity(geometry_msgs::PoseStamped subgoal);


private:
  ros::NodeHandle n;

  hl_helper hl_helper_;
  hl_constants hl_constants_;
  std::string mode;
  geometry_msgs::PoseStamped current_pose_in_world_frame;
  geometry_msgs::PoseStamped current_pose_in_target_frame;
  geometry_msgs::PoseStamped target_in_robot_frame;
  nav_msgs::Path current_path;
};


#endif
