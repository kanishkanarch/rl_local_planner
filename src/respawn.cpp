#include <roscpp.h>
#include <ros/>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "respawner");
	ros::NodeHandle nh;
	ros::publisher pub = nh.advertise<>
	return 0;
}
