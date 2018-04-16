#ifndef _HL_POSE_ESTIMATOR_
#define _HL_POSE_ESTIMATOR_

class hl_pose_estimator
{
public:
    hl_pose_estimator();
    ~hl_pose_estimator();

    ros::Subscriber robot_pose_from_ground_truth_sub;
    ros::Publisher estimated_pose_pub;


};


#endif
