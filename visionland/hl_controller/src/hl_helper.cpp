#include "hl_controller/hl_helper.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

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

geometry_msgs::Vector3 hl_helper::QuaternionToEuler(double qx, double qy, double qz, double qw) // Z-Y-X Euler angles
{
    struct{
	double x; 
	double y; 
	double z;
	double w;
	}q;

	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;
    geometry_msgs::Vector3 euler;
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;

    double TEST = q.w*q.y - q.x*q.z;

    if (TEST < -Threshold) // 奇异姿态,俯仰角为±90°
    {
        euler.z = 2 * (double)atan2(q.x, q.w); // yaw

        euler.y = -1.5708; // pitch

        euler.x = 0; // roll

    }
    else if (TEST > Threshold) // 奇异姿态,俯仰角为±90°
    {
        euler.z = -2 * (double)atan2(q.x, q.w); // yaw

        euler.y =  1.5708; // pitch

        euler.x = 0; // roll

    }
    else
    {
        euler.x = atan2(2 * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
        euler.y = asin(-2 * (q.x*q.z - q.w*q.y));
        euler.z = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    }
        
    return euler;
}
