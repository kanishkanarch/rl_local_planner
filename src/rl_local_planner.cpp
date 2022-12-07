#include "rl_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rl_local_planner_ns::rl_local_planner, nav_core::BaseLocalPlanner)

	namespace rl_local_planner_ns{
		rl_local_planner::rl_local_planner() 
		{
			odom_sub = nh.subscribe("/odom", 1, &rl_local_planner::odom_callback, this);
			scan_sub = nh.subscribe("/scan", 1, &rl_local_planner::scan_callback, this);
			respawner = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
			respawner_data.model_name = "turtlebot3_burger";
			respawner_data.pose.position.x = -2.0;
			respawner_data.pose.position.y = -0.5;
			respawner_data.pose.orientation.w = 1.0;
			respawner_srv.request.model_state = respawner_data;
			respawner.call(respawner_srv);
			time_prev = time(0);
			time_now = time(0);
		}


		rl_local_planner::~rl_local_planner() {}
		bool rl_local_planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
		{
			time_now = time(0);
			if (time_now - time_prev > 5)
			{
				respawner.call(respawner_srv);
				time_prev = time(0);
				time_now = time(0);
			}
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

		void rl_local_planner::odom_callback(const nav_msgs::Odometry odom_msg)
		{
			odom = odom_msg;
		}

		void rl_local_planner::scan_callback(const sensor_msgs::LaserScan scan_msg)
		{
			scan = scan_msg;
		}
	};
