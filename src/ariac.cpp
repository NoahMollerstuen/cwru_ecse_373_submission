#include "ros/ros.h"
#include "std_srvs/Trigger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ariac_interface");

  ros::NodeHandle n;
  
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  std_srvs::Trigger begin_comp;
  
  // Start the competition
  int service_call_succeeded;
  service_call_succeeded = begin_client.call(begin_comp);
  
  if(!service_call_succeeded) {
	  ROS_ERROR("Competition service call failed!");
	  return 1;
  }
  
  if(!begin_comp.response.success) {
	  ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  } else {
	  ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
  }
  

  return 0;
}

