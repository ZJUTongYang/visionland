#ifndef _HL_CONSTANTS_
#define _HL_CONSTANTS_

class hl_constants
{
public:
  hl_constants();
  ~hl_constants();
  double getMAXV(){return max_v;}
  double getHeight_of_goal_position(){return height_of_goal_position;}
  double getMin_height_tag_out_of_view(){return min_height_tag_out_of_view;}
private:
  double max_v;
  double max_vx;
  double max_vy;
  double max_vz;
  double height_of_goal_position;
  double min_height_tag_out_of_view;
};
#endif
