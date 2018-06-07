#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Dense.h"
#include "hl_tag_estimator.h"

hl_tag_estimator::hl_tag_estimator()
{

//	amymark position pub
	amymark_pose_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);

	set_amymark_pose();
}

hl_movetag::~hl_movetag()
{
}


using namespace std;
using namespace Eigen;
int main()
{
	MatrixXf A1 = MatrixXf::Random(3, 2);
	std::cout << "Here is the matrix A:\n" << A1 << std::endl;
	VectorXf b1 = VectorXf::Random(3);
	std::cout << "Here is the right hand side b:\n" << b1 << std::endl;
	//jacobiSvd 方式:Slow (but fast for small matrices)
	std::cout << "The least-squares solution is:\n"
	<< A1.jacobiSvd(ComputeThinU | ComputeThinV).solve(b1) << std::endl;
	//colPivHouseholderQr方法:fast
	std::cout << "The least-squares solution is:\n"
	<< A1.colPivHouseholderQr().solve(b1) << std::endl;
}



void hl_movetag::set_amymark_pose()
{
	ros::Time amy_t0 = ros::Time::now();	
	

while(1)
	{
		ros::Time amy_t1 = ros::Time::now();
		ros::Duration delta_t(amy_t1 - amy_t0);
		
		amy_pose.model_name = "amy_mark";
		//amy_pose.pose.position.x = -45.0 + delta_t.toSec() * 0.1;
                //amymark only move in -5<x<5, y=1, z=0.
		//if velocity of amymark is 0.2, the loop time is 50s.
		//For example , if delta_t.toSec() = ros::Time::now() = 65.3s;
		//then 65.3 - int(65.3) / 10 * 10 = 5.3s
		if(((int(delta_t.toSec() / 10)) % 2) == 1) //5->-5
			amy_pose.pose.position.x = 5.0 - (delta_t.toSec() - int(delta_t.toSec()) / 10 * 10) * 1;
		if(((int(delta_t.toSec() / 10)) % 2) == 0) //-5->5
			amy_pose.pose.position.x = -5.0 + (delta_t.toSec() - int(delta_t.toSec()) / 10 * 10) * 1;

		amy_pose.pose.position.y = 0;
		amy_pose.pose.position.z = 0;
		amy_pose.pose.orientation.x = 0;
		amy_pose.pose.orientation.y = 0;
		amy_pose.pose.orientation.z = 0;
		amy_pose.pose.orientation.w = 1;	
		amymark_pose_pub.publish(amy_pose);
	}

}
