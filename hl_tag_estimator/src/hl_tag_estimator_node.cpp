#include <iostream>
#include "ros/ros.h"
#include "Eigen/Dense.h"

#include "hl_tag_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hl_tag_estimator");

    hl_tag_estimator hl;

    ros::spin();
    return 0;
}
