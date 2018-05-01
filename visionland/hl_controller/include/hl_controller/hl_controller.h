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
#include "tf/transform_listener.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"


#define VTOL_STATE_MC 3


class hl_controller
{

public:
  hl_controller();
  ~hl_controller(){}
  void init_motors();

  
  ros::Subscriber tag_detection_sub;
  //YT get the robot pose (from ground_truth or pose_estimator)
  ros::Subscriber state_sub;
  ros::Subscriber local_pos_sub;

  ros::Subscriber mode_change_sub;
  ros::Publisher local_pos_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher setpoint_attitude_pub;


  ros::ServiceClient client;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  void state_Callback(const mavros_msgs::State::ConstPtr& msg);
  void tag_detections_Callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);
  void local_pos_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void mode_change_Callback(const std_msgs::String::ConstPtr& msg);
  void executeCB();
  void setspeed(double vx, double vy, double vz);
  void set_velocity(geometry_msgs::PoseStamped& subgoal);
  
    void enable_offboard();

private:
  ros::NodeHandle n;

  hl_helper hl_helper_;
  hl_constants hl_constants_;
  tf::TransformListener transform_listener_;
  tf::StampedTransform target_in_robot_frame_transform;
  std::string hl_mode;
  geometry_msgs::PoseStamped current_pose_in_world_frame;
//  geometry_msgs::PoseStamped current_pose_in_target_frame;
  geometry_msgs::PoseStamped target_in_virtual_camera_frame;
  geometry_msgs::PoseStamped target_in_camera_frame;
  geometry_msgs::PoseStamped target_in_robot_frame;//YT will be replaced by tf_transform
  nav_msgs::Path current_path;
  ros::Time last_time_get_tag_detections;
  geometry_msgs::PoseStamped setpoint_attitude;
  
    //ZYX for pixhawk
    mavros_msgs::State px4_state;
    geometry_msgs::PoseStamped current_position;

};


#endif
