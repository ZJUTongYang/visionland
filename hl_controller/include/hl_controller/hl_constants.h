#ifndef _HL_CONSTANTS_
#define _HL_CONSTANTS_

class hl_constants
{
public:
  hl_constants();
  ~hl_constants();
  double getMAXV(){return max_v;}

private:
  double max_v;
  double max_vx;
  double max_vy;
  double max_vz;
};
#endif
