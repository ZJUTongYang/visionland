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

    //   - estimate tag pose in camera frame
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is down and z is straight
    //     ahead
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is up and z is towards you (out of the tag).

std::cout<<"tag_detection_callback"<<std::endl;

//tag in camera frame
    target_in_camera_frame.header.stamp = msg->detections.at(0).pose.header.stamp;
    target_in_camera_frame.header.frame_id = msg->detections.at(0).pose.header.frame_id;
    target_in_camera_frame.pose.position.x = msg->detections.at(0).pose.pose.pose.position.x;
    target_in_camera_frame.pose.position.y = msg->detections.at(0).pose.pose.pose.position.y;
    target_in_camera_frame.pose.position.z = msg->detections.at(0).pose.pose.pose.position.z;
    target_in_camera_frame.pose.orientation.x = msg->detections.at(0).pose.pose.pose.orientation.x;
    target_in_camera_frame.pose.orientation.y = msg->detections.at(0).pose.pose.pose.orientation.y;
    target_in_camera_frame.pose.orientation.z = msg->detections.at(0).pose.pose.pose.orientation.z;
    target_in_camera_frame.pose.orientation.w = msg->detections.at(0).pose.pose.pose.orientation.w;
            std::cout << "YT:check the target in camera frame: [" 
                      << target_in_camera_frame.pose.position.x
                      << ", "
                      << target_in_camera_frame.pose.position.y
                      << ", "
                      << target_in_camera_frame.pose.position.z
                      << "] " << std::endl;
//tag in robot frame
//q1 = xi + yi + zk + w
//camera optical frame --> robot frame = Z --> -90 degrees \ Y --> 0 degree \ X --> 180 degrees
//q2=q_cameraoptical2robot = [0,0,-0.707,0.707] = 0w + 0i -0.707j + 0.707k
//q = q2*q1 =  - 0.707 * (y + z) * i + 0.707 * (x - w) * j + 0.707 * (x + w) * k + 0.707 * (y - z)
//offset is calculate in robot frame.
float offset_in_UAV_frame_x = 0.0;
float offset_in_UAV_frame_y = 0.0;
float offset_in_UAV_frame_z = 0.0;
    target_in_robot_frame.pose.position.x = -target_in_camera_frame.pose.position.y + offset_in_UAV_frame_x;
    target_in_robot_frame.pose.position.y = -target_in_camera_frame.pose.position.x + offset_in_UAV_frame_y;
    target_in_robot_frame.pose.position.z = -target_in_camera_frame.pose.position.z + offset_in_UAV_frame_z;
    target_in_robot_frame.pose.orientation.x = -0.707 * (target_in_camera_frame.pose.orientation.y + target_in_camera_frame.pose.orientation.z);
    target_in_robot_frame.pose.orientation.y = 0.707 * (target_in_camera_frame.pose.orientation.x - target_in_camera_frame.pose.orientation.w);
    target_in_robot_frame.pose.orientation.z = 0.707 * (target_in_camera_frame.pose.orientation.x + target_in_camera_frame.pose.orientation.w);
    target_in_robot_frame.pose.orientation.w = 0.707 * (target_in_camera_frame.pose.orientation.y-target_in_camera_frame.pose.orientation.z);
            std::cout << "YT:check the target in robot frame: [" 
                      << target_in_robot_frame.pose.position.x
                      << ", "
                      << target_in_robot_frame.pose.position.y
                      << ", "
                      << target_in_robot_frame.pose.position.z
                      << "] " << std::endl;
  
/* 20180425 */
//camera in tag frame --> tag in robot frame
/* 
//camera in tag frame
    current_pose_in_target_frame.header.stamp = msg->detections.at(0).pose.header.stamp;
    current_pose_in_target_frame.header.frame_id = msg->detections.at(0).pose.header.frame_id;
    current_pose_in_target_frame.pose.position.x = msg->detections.at(0).pose.pose.pose.position.x;
    current_pose_in_target_frame.pose.position.y = msg->detections.at(0).pose.pose.pose.position.y;
    current_pose_in_target_frame.pose.position.z = msg->detections.at(0).pose.pose.pose.position.z;
    current_pose_in_target_frame.pose.orientation.x = msg->detections.at(0).pose.pose.pose.orientation.x;
    current_pose_in_target_frame.pose.orientation.y = msg->detections.at(0).pose.pose.pose.orientation.y;
    current_pose_in_target_frame.pose.orientation.z = msg->detections.at(0).pose.pose.pose.orientation.z;
    current_pose_in_target_frame.pose.orientation.w = msg->detections.at(0).pose.pose.pose.orientation.w;
            std::cout << "YT:check the current_pose_in_target_frame: [" 
                      << current_pose_in_target_frame.pose.position.x
                      << ", "
                      << current_pose_in_target_frame.pose.position.y
                      << ", "
                      << current_pose_in_target_frame.pose.position.z
                      << "] " << std::endl;

	//quaternions --> matrix rotation 
	//R = [r11 r12 r13
	//     r21 r22 r23
	//     r31 r32 r33]
float px = current_pose_in_target_frame.pose.position.x;
float py = current_pose_in_target_frame.pose.position.y;
float pz = current_pose_in_target_frame.pose.position.z;
float qx = current_pose_in_target_frame.pose.orientation.x;
float qy = current_pose_in_target_frame.pose.orientation.y;
float qz = current_pose_in_target_frame.pose.orientation.z;
float qw = current_pose_in_target_frame.pose.orientation.w;

float r11 = 1 - 2 * qy * qy - 2 * qz * qz;
float r12 = 2 * (qx * qy - qz * qw);
float r13 = 2 * (qx * qz + qy * qw);
float r21 = 2 * (qx * qy + qz * qw);
float r22 = 1 - 2 * qx * qx - 2 * qz * qz;
float r23 = 2 * (qy * qz - qx * qw);
float r31 = 2 * (qx * qz - qy * qw);
float r32 = 2 * (qy * qz + qx * qw);
float r33 = 1 - 2 * qx * qx - 2 * qy * qy;

//R^T = [r11 r21 r31
//       r12 r22 r32
//       r13 r23 r33]
//P_3*1 = - R^T .* p_3*1

float Px = -(r11 + r12 + r13) * px;
float Py = -(r21 + r22 + r23) * py;
float Pz = -(r31 + r32 + r33) * pz;

float Qw = 0.5 * sqrt (1 + r11 + r22 + r33);
float Qx = (r23 - r32) / 4 / Qw;
float Qy = (r31 - r13) / 4 / Qw;
float Qz = (r12 - r21) / 4 / Qw;

//tag in camera frame
    target_in_camera_frame.pose.position.x = Px;
    target_in_camera_frame.pose.position.y = Py;
    target_in_camera_frame.pose.position.z = Pz;
    target_in_camera_frame.pose.orientation.x = Qx;
    target_in_camera_frame.pose.orientation.y = Qy;
    target_in_camera_frame.pose.orientation.z = Qz;
    target_in_camera_frame.pose.orientation.w = Qw;
            std::cout << "YT:check the target in camera frame: [" 
                      << target_in_camera_frame.pose.position.x
                      << ", "
                      << target_in_camera_frame.pose.position.y
                      << ", "
                      << target_in_camera_frame.pose.position.z
                      << "] " << std::endl;
//tag in robot frame
//q1 = xi + yi + zk + w
//camera optical frame --> robot frame = Z --> -90 degrees \ Y --> 0 degree \ X --> 180 degrees
//q2=q_cameraoptical2robot = [0,0,-0.707,0.707] = 0w + 0i -0.707j + 0.707k
//q = q2*q1 =  - 0.707 * (y + z) * i + 0.707 * (x - w) * j + 0.707 * (x + w) * k + 0.707 * (y - z)
//offset is calculate in robot frame.
float offset_in_UAV_frame_x = 0.0;
float offset_in_UAV_frame_y = 0.0;
float offset_in_UAV_frame_z = 0.0;
    target_in_robot_frame.pose.position.x = -target_in_camera_frame.pose.position.y + offset_in_UAV_frame_x;
    target_in_robot_frame.pose.position.y = -target_in_camera_frame.pose.position.x + offset_in_UAV_frame_y;
    target_in_robot_frame.pose.position.z = -target_in_camera_frame.pose.position.z + offset_in_UAV_frame_z;
    target_in_robot_frame.pose.orientation.x = -0.707 * (target_in_camera_frame.pose.orientation.y + target_in_camera_frame.pose.orientation.z);
    target_in_robot_frame.pose.orientation.y = 0.707 * (target_in_camera_frame.pose.orientation.x - target_in_camera_frame.pose.orientation.w);
    target_in_robot_frame.pose.orientation.z = 0.707 * (target_in_camera_frame.pose.orientation.x + target_in_camera_frame.pose.orientation.w);
    target_in_robot_frame.pose.orientation.w = 0.707 * (target_in_camera_frame.pose.orientation.y-target_in_camera_frame.pose.orientation.z);
            std::cout << "YT:check the target in robot frame: [" 
                      << target_in_robot_frame.pose.position.x
                      << ", "
                      << target_in_robot_frame.pose.position.y
                      << ", "
                      << target_in_robot_frame.pose.position.z
                      << "] " << std::endl;

//camera in tag frame --> tag in robot frame
*/
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

