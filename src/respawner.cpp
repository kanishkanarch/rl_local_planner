#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo/SetModelState.h"
#include <random>

int main (int argc, char **argv)
{

  ros::init(argc, argv, "respawner");
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  
  std::random_device                  rand_dev;
  std::mt19937                        generator(rand_dev());
  std::uniform_int_distribution<>  distr(0,20);
  for(int n=0; n<40; ++n)
        std::cout << distr(generator) << ' '; // generate numbers
  
   geometry_msgs::Pose start_pose;
   start_pose.position.x = 0.0;
   start_pose.position.y = 0.0;
   start_pose.position.z = 0.0;
   start_pose.orientation.x = 0.0;
   start_pose.orientation.y = 0.0;
   start_pose.orientation.z = 0.0;
   start_pose.orientation.w = 0.0;


   gazebo_msgs::ModelState modelstate;
   modelstate.model_name = (std::string) "turtlebot3_burger";
   modelstate.reference_frame = (std::string) "world";
   modelstate.pose = start_pose;
        
   ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
   gazebo_msgs::SetModelState setmodelstate;
   setmodelstate.request.model_state = modelstate;     
   
   return 0;
}
