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

geometry_msgs::PoseStamped hl_helper::new_PoseStamped_no_header(double pos_x, double pos_y, double pos_z, double ori_x, double ori_y, double ori_z, double ori_w)
{
    geometry_msgs::PoseStamped temp;
    temp.header.stamp = ros::Time::now();
//    temp.header.frame_id = "";
    temp.pose.position.x = pos_x;
    temp.pose.position.y = pos_y;
    temp.pose.position.z = pos_z;
    
    temp.pose.orientation.x = ori_x;
    temp.pose.orientation.y = ori_y;
    temp.pose.orientation.z = ori_z;
    temp.pose.orientation.w = ori_w;

    return temp;
}
