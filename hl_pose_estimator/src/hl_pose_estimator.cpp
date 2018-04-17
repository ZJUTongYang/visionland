#include "hl_pose_estimator.h"
#include "geometry_msgs/PoseStamped.h"


hl_pose_estimator::hl_pose_estimator()
{
    robot_pose_from_ground_truth_sub = n.subscribe(
                       "/ground_truth_to_tf/pose", 1,
                       &hl_pose_estimator::ground_truth_Callback, this);
    estimated_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
                       "/hl_estimated_pose", 1);


}

hl_pose_estimator::~hl_pose_estimator()
{
}

void hl_pose_estimator::ground_truth_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose.pose.position.x = msg->pose.position.x;
  current_pose.pose.position.y = msg->pose.position.y;
  current_pose.pose.position.z = msg->pose.position.z;

  current_pose.pose.orientation.x = msg->pose.orientation.x;
  current_pose.pose.orientation.y = msg->pose.orientation.y;
  current_pose.pose.orientation.z = msg->pose.orientation.z;
  current_pose.pose.orientation.w = msg->pose.orientation.w;
  
  estimated_pose_pub.publish(current_pose);
}
