#ifndef _HL_POSE_ESTIMATOR_
#define _HL_POSE_ESTIMATOR_
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelState.h"

class hl_movetag
{
public:
    hl_movetag();
    ~hl_movetag();

  ros::Publisher amymark_pose_pub;
  void set_amymark_pose();
   
private:
    ros::NodeHandle n;
	gazebo_msgs::ModelState amy_pose;
};


#endif
