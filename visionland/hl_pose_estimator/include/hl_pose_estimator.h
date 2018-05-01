#ifndef _HL_POSE_ESTIMATOR_
#define _HL_POSE_ESTIMATOR_
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class hl_pose_estimator
{
public:
    hl_pose_estimator();
    ~hl_pose_estimator();

    void ground_truth_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber robot_pose_from_ground_truth_sub;
    ros::Publisher estimated_pose_pub;
    
private:
    ros::NodeHandle n;
    geometry_msgs::PoseStamped current_pose;
};


#endif
