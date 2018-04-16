#include <iostream>
#include "ros/ros.h"

#include "hl_pose_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hl_pose_estimator");

    hl_pose_estimator hl;

    ros::spin();
    return 0;
}
