#include <iostream>
#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"

#include "hl_movetag.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hl_movetag");

    hl_movetag hl;

    ros::spin();
    return 0;
}
