#include "hl_controller/hl_helper.h"
#include "geometry_msgs/PoseStamped.h"

hl_helper::hl_helper()
{
}

hl_helper::~hl_helper()
{
}

void cout_PoseStamped(const geometry_msgs::PoseStamped pose)
{
    std::cout << "[" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << "], [" 
    << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", " << pose.pose.orientation.z << ", " << pose.pose.orientation.w << "]" << std::endl;

}
