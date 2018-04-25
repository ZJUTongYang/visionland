#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "hl_controller/hl_controller.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "hl_controller/hl_helper.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"


hl_controller::hl_controller()
{
    state_sub = n.subscribe("mavros/state", 1, &hl_controller::state_Callback, this);
    local_pos_sub = n.subscribe("mavros/local_position/pose", 1, &hl_controller::robot_pose_Callback, this);
    local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);


    tag_detection_sub = n.subscribe("/tag_detections", 1, &hl_controller::tag_detections_Callback, this);
//    robot_pose_sub = n.subscribe("/hl_estimated_pose", 1, &hl_controller::robot_pose_Callback, this);
    mode_change_sub = n.subscribe("/hl_change_mode", 1, &hl_controller::mode_change_Callback, this);
    cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    path_pub = n.advertise<nav_msgs::Path>("/hl_global_path", 1);

    client = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



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

    //YT for hector, enabling motors
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

    //ZYX for pixhawk, waiting for FCU connection
    ros::Rate rate(20.0);
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    double origin_x = 0, origin_y = 0, origin_z = 0;

    //send a few setpoints before starting
    for(unsigned int i = 0;i < 100;i++)
    {
   
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = 0;
        temp.pose.position.y = 0;
        temp.pose.position.z = 5;
//        temp.pose.orientation.x = 0;
//        temp.pose.orientation.y = 0;
//        temp.pose.orientation.z = 0;
//        temp.pose.orientation.w = 1;
        local_pos_pub.publish(temp);
	ROS_INFO("Position Control before");
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode offb_set_mode1;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode1.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    
    ros::Time last_request = ros::Time::now();

    //YT make sure current_state.mode is "OFFBOARD" and current_state.armed is "true"
    while(ros::ok()){
        bool arm_flag = 0;
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){        //(indigo)mode_sent-->success
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    arm_flag = 1;
                }
                last_request = ros::Time::now();
            }
        }

        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = 0;
        temp.pose.position.y = 0;
        temp.pose.position.z = 10;
//        temp.pose.orientation.x = 0;
//        temp.pose.orientation.y = 0;
//        temp.pose.orientation.z = 0;
//        temp.pose.orientation.w = 1;
        local_pos_pub.publish(temp);


	ros::spinOnce();
	rate.sleep();
        if(arm_flag ==1)
            break;
    }
    std::cout << "YT: enabled hector & pixhawk ! " <<  std::endl;
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

        //YT for pixhawk
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = 0;
        temp.pose.position.y = 0;
        temp.pose.position.z = 10;
//        temp.pose.orientation.x = 0;
//        temp.pose.orientation.y = 0;
//        temp.pose.orientation.z = 0;
//        temp.pose.orientation.w = 1;
        local_pos_pub.publish(temp);
	ROS_INFO("Position Control 10m");
    }
    if(mode == "stabilized")
    {
        geometry_msgs::TwistStamped cmd_vel_stamped;
        cmd_vel_stamped.twist.linear.x = 0;
        cmd_vel_stamped.twist.linear.y = 0;
        cmd_vel_stamped.twist.linear.z = 0;
        cmd_vel_stamped.twist.angular.x = 0;
        cmd_vel_stamped.twist.angular.y = 0;
        cmd_vel_stamped.twist.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel_stamped);
    }
    if(mode == "tracking")
    {
        //YT we want to stop at 2m above the target place

        set_velocity(target_in_robot_frame);
    }

    if(mode == "landing")
    {
        geometry_msgs::TwistStamped cmd_vel_stamped;
        cmd_vel_stamped.twist.linear.z = -hl_constants_.getMAXV();
        cmd_vel_pub.publish(cmd_vel_stamped);
    }      
    ros::Duration(0.1).sleep();
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
    geometry_msgs::TwistStamped cmd_vel_stamped;

    cmd_vel_stamped.twist.linear.x = hl_constants_.getMAXV() * goal.pose.position.x / scale_factor;
    cmd_vel_stamped.twist.linear.y = hl_constants_.getMAXV() * goal.pose.position.y / scale_factor;
//    cmd_vel.linear.z = 0;
    cmd_vel_stamped.twist.linear.z = hl_constants_.getMAXV() * goal.pose.position.z / scale_factor;
    cmd_vel_stamped.twist.angular.x = 0;
    cmd_vel_stamped.twist.angular.y = 0;
    cmd_vel_stamped.twist.angular.z = 0;
    std::cout << "YT: cmd_vel = [" << cmd_vel_stamped.twist.linear.x << ", " << cmd_vel_stamped.twist.linear.y << ", " << cmd_vel_stamped.twist.linear.z << "]"<< std::endl;
    cmd_vel_pub.publish(cmd_vel_stamped);
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

void hl_controller::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


