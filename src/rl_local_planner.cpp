#include <pluginlib/class_list_macros.h>
#include "rl_local_planner.h"

PLUGINLIB_EXPORT_CLASS(rl_local_planner_ns::rl_local_planner, nav_core::BaseLocalPlanner)

	namespace rl_local_planner_ns{
		rl_local_planner::rl_local_planner(){}

		bool rl_local_planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
		{
			cmd_vel.angular.z = 0.1;
		}

		void rl_local_planner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
		{
		}
		bool rl_local_planner::isGoalReached()
		{
		}
		bool rl_local_planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
		{
		}
	};
