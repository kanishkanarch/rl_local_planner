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
			odom_sub = nh.subscribe("/odom", 1, &rl_local_planner::odom_callback, this);
			scan_sub = nh.subscribe("/scan", 1, &rl_local_planner::scan_callback, this);
			imu_sub = nh.subscribe("/imu", 1, &rl_local_planner::imu_callback, this);
			respawner = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
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

		void rl_local_planner::odom_callback(const nav_msgs::Odometry odom_msg)
		{
			odom = odom_msg;
		}

		void rl_local_planner::scan_callback(const sensor_msgs::LaserScan scan_msg)
		{
			scan = scan_msg;
		}

		void rl_local_planner::imu_callback(const sensor_msgs::Imu imu_msg)
		{
			while (ros::ok())
			{
				respawner.call(respawner_srv);
				ros::spinOnce();
				sleep(5000);

			}
		}

	};
