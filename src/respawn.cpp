#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "respawner");
	ros::NodeHandle n;
	ros::Rate looprate(0.1);

	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
	std_srvs::Empty srv_data;
	while(ros::ok())
	{
		client.call(srv_data);
		looprate.sleep();
	}
	return 0;
}
