#ifndef RL_LOCAL_PLANNER_H
#define RL_LOCAL_PLANNER_H
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <unistd.h>
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace rl_local_planner_ns{
	class rl_local_planner:public nav_core::BaseLocalPlanner
	{
		public:
			rl_local_planner();
			~rl_local_planner();
			bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
			void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
			bool isGoalReached();
			bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
			ros::NodeHandle nh;
			ros::Subscriber odom_sub;
			ros::Subscriber scan_sub;
			ros::Subscriber imu_sub;
			nav_msgs::Odometry odom;
			sensor_msgs::LaserScan scan;
			ros::ServiceClient respawner;
			gazebo_msgs::SetModelState respawner_srv;
			gazebo_msgs::ModelState respawner_data;
			respawner_data.model_name = "turtlebot3_burger";
			respawner_data.pose.position.x = -2.0;
			respawner_data.pose.position.y = -0.5;
			respawner_data.pose.orientation.w = 1.0;
			respawner_srv.request.model_state = respawner_data;
			void odom_callback(const nav_msgs::Odometry odom_msg);
			void scan_callback(const sensor_msgs::LaserScan scan_msg);
			void imu_callback(const sensor_msgs::Imu imu_msg);
	};
};
#endif
//PLUGINLIB_EXPORT_CLASS(rl_local_planner_ns::rl_local_planner, nav_core::BaseLocalPlanner)
