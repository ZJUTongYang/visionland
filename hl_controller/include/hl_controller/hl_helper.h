#ifndef _HL_HELPER_
#define _HL_HELPER_

#include "geometry_msgs/PoseStamped.h"

class hl_helper
{
public:
  hl_helper();
  ~hl_helper();
  void cout_PoseStamped(const std::string foreword, const geometry_msgs::PoseStamped& pose, bool show_pos = true, bool show_quaternion = false);
  geometry_msgs::PoseStamped new_PoseStamped_no_header(double pos_x, double pos_y, double pos_z, double ori_x, double ori_y, double ori_z, double ori_w);

};
#endif
