#ifndef _HL_HELPER_
#define _HL_HELPER_

#include "geometry_msgs/PoseStamped.h"

class hl_helper
{
public:
  hl_helper();
  ~hl_helper();
  void cout_PoseStamped(const geometry_msgs::PoseStamped pose);

};
#endif
