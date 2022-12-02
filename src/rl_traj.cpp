#include <pluginlib/class_list_macros.h>
#include "rl_traj.h"

PLUGINLIB_EXPORT_CLASS(rl_traj, nav_core::BaseLocalPlanner)

rl_traj::rl_traj(){}

bool rl_traj::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
	cmd_vel.angular.z = 0.1;
}

void rl_traj::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
}
bool rl_traj::isGoalReached()
{
}
bool rl_traj::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
}
