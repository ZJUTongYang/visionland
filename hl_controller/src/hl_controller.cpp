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
    local_pos_sub = n.subscribe("mavros/local_position/pose", 1, &hl_controller::local_pos_Callback, this);
    local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    tag_detection_sub = n.subscribe("/tag_detections", 1, &hl_controller::tag_detections_Callback, this);
    mode_change_sub = n.subscribe("/hl_change_mode", 1, &hl_controller::mode_change_Callback, this);
    cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    client = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    hl_mode = "waiting";

    init_motors();
  
    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        executeCB();   
        
        if((hl_mode == "takeoff")&&(current_pose_in_world_frame.pose.position.z > 5))
            hl_mode = "stabilized";//YT cannot convert to tracking directly

        if((hl_mode =="tracking") &&
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

            hl_mode = "landing";
        }
        



        ros::Time t = ros::Time::now();
        ros::Duration d(t - last_time_get_tag_detections);

        if((hl_mode == "waiting") && (d.toSec() < 0.4))
        {
            enable_offboard();    
	    hl_mode = "tracking";
        }

        if((hl_mode == "tracking")&&(d.toSec() > 0.5))
        {
            hl_mode = "landing";
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//YT before init motors we need some time to wait for other ROS module
void hl_controller::init_motors()
{
    //for pixhawk, waiting for FCU connection
    ros::Rate rate(20.0);
    while(ros::ok() && px4_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    double origin_x = 0, origin_y = 0, origin_z = 0;


    std::cout << "YT: enabled pixhawk ! " <<  std::endl;
}

void hl_controller::enable_offboard()
{
    ros::Rate rate(20.0);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();
     //YT make sure px4_state.mode is "OFFBOARD" and px4_state.armed is "true"
    while(ros::ok()){
        bool arm_flag = 0;
        if( px4_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){        //(indigo)mode_sent-->success
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !px4_state.armed &&
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
        temp.pose.position.x = current_pose_in_world_frame.pose.position.x;
        temp.pose.position.y = current_pose_in_world_frame.pose.position.y;
        temp.pose.position.z = current_pose_in_world_frame.pose.position.z;
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
    std::cout << "YT: set pixhawk to offboard mode!" << std::endl;
}


void hl_controller::executeCB()
{
    std::cout << "YT: hl_mode = " << hl_mode << ", ";
    hl_helper_.cout_PoseStamped("target", target_in_robot_frame, true, false);

    if(hl_mode == "takeoff")
    {

        //YT for pixhawk
        local_pos_pub.publish(hl_helper_.new_PoseStamped_no_header(0, 0, 5, 0, 0, 0, 1));
	ROS_INFO("Position Control 5m");
    }
    if(hl_mode == "stabilized")
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
    if(hl_mode == "tracking")
    {
        //YT we want to stop at 2m above the target place

        set_velocity(target_in_robot_frame);
    }

    if(hl_mode == "landing")
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
    cmd_vel_stamped.twist.linear.z = hl_constants_.getMAXV() * goal.pose.position.z / scale_factor;
    cmd_vel_stamped.twist.angular.x = 0;
    cmd_vel_stamped.twist.angular.y = 0;
    cmd_vel_stamped.twist.angular.z = 0;
    std::cout << "YT: cmd_vel = [" << cmd_vel_stamped.twist.linear.x << ", " << cmd_vel_stamped.twist.linear.y << ", " << cmd_vel_stamped.twist.linear.z << "]"<< std::endl;
    cmd_vel_pub.publish(cmd_vel_stamped);
    goal.pose.position.z -= 2;
}

void hl_controller::local_pos_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
    if((msg->data == "tracking") || (msg->data == "landing"))
    {
        std::cout << "YT: receive new flight hl_mode" << std::endl;
        hl_mode = msg->data;
    }
    else
    {
        std::cout << "YT: invalid hl_mode" << std::endl;
    }
}

void hl_controller::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    px4_state = *msg;
}


