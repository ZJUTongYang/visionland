#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
//#include "apriltags2_ros/AprilTagDetectionArray.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "hector_uav_msgs/EnableMotors.h"
#include "hl_controller/hl_controller.h"
//void tag_detections_Callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
//{
//  std::cout << "YT: plane's height: " << msg->detections.at(0).pose.pose.pose.position.z << std::endl;
	//ROS_INFO("I heard: [%s]",msg->data.c_str());
//}

int main(int argc,char **argv)
{
   ros::init(argc, argv, "hl_controller");
//   ros::NodeHandle n;
//   ros::Subscriber tag_detections_sub;
//   ros::Publisher pose_pub;
//   ros::ServiceClient client;
   
//   tag_detections_sub = n.subscribe("/tag_detections", 100, tag_detections_Callback);
//   pose_pub = n.advertise<geometry_msgs::PoseStamped>("/command/pose", 100);
//   client = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

   hl_controller hl;

   //YT enable motor first
//   ros::Time t0 = ros::Time::now();
//   while(1)
//   {
//      ros::Time t1 = ros::Time::now();
//      ros::Duration d(t1 - t0);
//      if(d.toSec() > 1)
//         break;
//   }
//   hector_uav_msgs::EnableMotors srv;
//   srv.request.enable = true;   
//   client.call(srv);

//   ros::Rate loop_rate(10);
//   int count = 0;
//   while (ros::ok())
//   {      
//      //client.call(srv);
//
//      geometry_msgs::PoseStamped pose_sp;
//      pose_sp.pose.position.x = 2.0;
//      pose_sp.pose.position.y = 2.0;
//      pose_sp.pose.position.z = 10.0;
//      
//      std::cout << "hl666" << std::endl;
//      pose_pub.publish(pose_sp);
//      ros::spinOnce();
//     loop_rate.sleep();
//     ++count;
//   }

   ros::spin();
   return 0;
}
