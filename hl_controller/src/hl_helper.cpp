#include "hl_controller/hl_helper.h"
#include "geometry_msgs/PoseStamped.h"

hl_helper::hl_helper()
{
}

hl_helper::~hl_helper()
{
}

void hl_helper::cout_PoseStamped(const std::string foreword, const geometry_msgs::PoseStamped& pose, bool show_pos, bool show_quaternion)
{
    std::cout << foreword;
    if(show_pos)
    {
    std::cout << ": [" 
              << pose.pose.position.x 
              << ", " 
              << pose.pose.position.y 
              << ", " 
              << pose.pose.position.z 
              << "]";
    }
    if(show_quaternion)
    {
        std::cout << ", [" 
                  << pose.pose.orientation.x 
                  << ", " 
                  << pose.pose.orientation.y 
                  << ", " 
                  << pose.pose.orientation.z 
                  << ", " 
                  << pose.pose.orientation.w 
                  << "]";
    }
    std::cout << std::endl;
}
