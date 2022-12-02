#include <nav_core/base_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

class rl_traj:public nav_core::BaseLocalPlanner
{
public:
	rl_traj();
	~rl_traj();
	bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
	void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
	bool isGoalReached();
	bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
};

PLUGINLIB_EXPORT_CLASS(rl_traj, nav_core::BaseLocalPlanner)
