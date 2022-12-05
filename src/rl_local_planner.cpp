#include "rl_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>

PLUGINLIB_EXPORT_CLASS(rl_local_planner_ns::rl_local_planner, nav_core::BaseLocalPlanner)

	namespace rl_local_planner_ns{
		rl_local_planner::rl_local_planner() 
		{
			initialize();
		}

               
               rl_local_planner::~rl_local_planner() {}
		bool rl_local_planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
		{
			cmd_vel.linear.x=0.1;
			cmd_vel.angular.z=0.2;
			return true;
		}

		void rl_local_planner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
		{
		}
		bool rl_local_planner::isGoalReached()
		{
		  return false;
		}
		bool rl_local_planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
		{ return true;
		}
		
		
	};
