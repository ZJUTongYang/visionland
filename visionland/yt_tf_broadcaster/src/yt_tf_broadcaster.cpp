#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

bool update;
nav_msgs::Odometry odom;


void tfCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;

    tf::Transform transform;
//    std::cout << "YT: size of tf = " << msg->name.size() << std::endl;
    for(unsigned int i = 1 ; i < (msg->name.size()) ; i++)
    {
    	transform.setOrigin(tf::Vector3(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z));
    	transform.setRotation(tf::Quaternion(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w));
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->name.at(0), msg->name.at(i)));
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "yt_tf_broadcaster");
    ros::NodeHandle nh;
//    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1, tfCallback);

    ros::Rate r(100);
    while(1)
    {
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
