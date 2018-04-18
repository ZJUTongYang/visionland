#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "hl_controller/hl_controller.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "hl_controller/hl_helper.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

hl_controller::hl_controller()
{
    tag_detection_sub = n.subscribe("/tag_detections", 1, &hl_controller::tag_detections_Callback, this);
    robot_pose_sub = n.subscribe("/hl_estimated_pose", 1, &hl_controller::robot_pose_Callback, this);
    mode_change_sub = n.subscribe("/hl_change_mode", 1, &hl_controller::mode_change_Callback, this);
//    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/command/pose", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    path_pub = n.advertise<nav_msgs::Path>("/hl_global_path", 1);

    client = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

    mode = "takeoff";

    init_motors();
  
    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        executeCB();   
        
        if((mode == "takeoff")&&(current_pose_in_world_frame.pose.position.z > 10))
            mode = "stabilized";//YT cannot convert to tracking directly

        if((mode =="tracking") &&
           (fabs(target_in_robot_frame.pose.position.x) < 0.1)&&
           (fabs(target_in_robot_frame.pose.position.y) < 0.1)&&
           (current_pose_in_world_frame.pose.position.z < 3))
        {
            std::cout << "YT:check the target_in_robot_frame: [" 
                      << target_in_robot_frame.pose.position.x
                      << ", "
                      << target_in_robot_frame.pose.position.y
                      << ", "
                      << target_in_robot_frame.pose.position.z
                      << "] " << std::endl;

            mode = "landing";
        }

        ros::Time t = ros::Time::now();
        ros::Duration d(t - last_time_get_tag_detections);
        if((mode == "tracking")&&(d.toSec() > 0.5))
        {
            mode = "landing";
        }

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

void hl_controller::executeCB()
{
    std::cout << "YT:mode = " << mode 
              << ", target: [" 
              << target_in_robot_frame.pose.position.x
              << ", "
              << target_in_robot_frame.pose.position.y
              << ", "
              << target_in_robot_frame.pose.position.z
              << "]" << std::endl;

    if(mode == "takeoff")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.z = 1;
        cmd_vel_pub.publish(cmd_vel);
    }
    if(mode == "stabilized")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel);
    }
    if(mode == "tracking")
    {
        //YT we want to stop at 2m above the target place

        set_velocity(target_in_robot_frame);
    }

    if(mode == "landing")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.z = -hl_constants_.getMAXV();
        cmd_vel_pub.publish(cmd_vel);
    }      
}

void hl_controller::tag_detections_Callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    last_time_get_tag_detections = ros::Time::now();//refresh the latest time of image message
    //std::cout << "YT: find at least one target!" << std::endl;
    current_pose_in_target_frame.header.stamp = msg->detections.at(0).pose.header.stamp;
    current_pose_in_target_frame.header.frame_id = msg->detections.at(0).pose.header.frame_id;
    current_pose_in_target_frame.pose.position.x = msg->detections.at(0).pose.pose.pose.position.x;
    current_pose_in_target_frame.pose.position.y = msg->detections.at(0).pose.pose.pose.position.y;
    current_pose_in_target_frame.pose.position.z = msg->detections.at(0).pose.pose.pose.position.z;
    current_pose_in_target_frame.pose.orientation.x = msg->detections.at(0).pose.pose.pose.orientation.x;
    current_pose_in_target_frame.pose.orientation.y = msg->detections.at(0).pose.pose.pose.orientation.y;
    current_pose_in_target_frame.pose.orientation.z = msg->detections.at(0).pose.pose.pose.orientation.z;
    current_pose_in_target_frame.pose.orientation.w = msg->detections.at(0).pose.pose.pose.orientation.w;
 
    target_in_robot_frame.pose.position.x = -current_pose_in_target_frame.pose.position.x;
    target_in_robot_frame.pose.position.y = -current_pose_in_target_frame.pose.position.y;
    target_in_robot_frame.pose.position.z = -current_pose_in_target_frame.pose.position.z;
    //YT currently we do not introduce angles
    target_in_robot_frame.pose.orientation.x = -current_pose_in_target_frame.pose.orientation.x;
    target_in_robot_frame.pose.orientation.y = -current_pose_in_target_frame.pose.orientation.y;
    target_in_robot_frame.pose.orientation.z = -current_pose_in_target_frame.pose.orientation.z;
    target_in_robot_frame.pose.orientation.w = current_pose_in_target_frame.pose.orientation.w;
}

void hl_controller::set_velocity(geometry_msgs::PoseStamped& goal)
{
    double scale_factor;
    goal.pose.position.z += 2;
    scale_factor = sqrt(goal.pose.position.x * goal.pose.position.x + 
                        goal.pose.position.y * goal.pose.position.y + 
                        goal.pose.position.z * goal.pose.position.z);
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = hl_constants_.getMAXV() * goal.pose.position.x / scale_factor;
    cmd_vel.linear.y = hl_constants_.getMAXV() * goal.pose.position.y / scale_factor;
//    cmd_vel.linear.z = 0;
    cmd_vel.linear.z = hl_constants_.getMAXV() * goal.pose.position.z / scale_factor;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    std::cout << "YT: cmd_vel = [" << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.linear.z << "]"<< std::endl;
    cmd_vel_pub.publish(cmd_vel);
    goal.pose.position.z -= 2;
}

void hl_controller::robot_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_in_world_frame.pose.position.x = msg->pose.position.x;
    current_pose_in_world_frame.pose.position.y = msg->pose.position.y;
    current_pose_in_world_frame.pose.position.z = msg->pose.position.z;

    current_pose_in_world_frame.pose.orientation.x = msg->pose.orientation.x;
    current_pose_in_world_frame.pose.orientation.y = msg->pose.orientation.y;
    current_pose_in_world_frame.pose.orientation.z = msg->pose.orientation.z;
    current_pose_in_world_frame.pose.orientation.w = msg->pose.orientation.w;
}

void hl_controller::mode_change_Callback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "YT: receive new flight mode" << std::endl;
    mode = msg->data;
}

