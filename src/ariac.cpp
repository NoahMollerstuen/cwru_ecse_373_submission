#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"

// Vector to keep track of received orders
std::vector<osrf_gear::Order::ConstPtr> orders;

// Sevice client to query the location of specific materials
ros::ServiceClient material_locations_client;
osrf_gear::GetMaterialLocations material_locations_srv;

void ordersCallback(const osrf_gear::Order::ConstPtr& msg) {
  ROS_INFO("Order received: [%s]", msg->order_id.c_str());
  orders.push_back(msg);
  ROS_INFO("First product type: [%s]", msg->shipments[0].products[0].type.c_str());
  
  // Query the location of the desired type of part
  material_locations_srv.request.material_type = msg->shipments[0].products[0].type;
  int call_succeeded;
  call_succeeded = material_locations_client.call(material_locations_srv);
  
  if(!call_succeeded) {
	  ROS_ERROR("Call to /ariac/material_locations failed!");
	  return;
  }
  
  ROS_INFO("This product can be found in bin [%s]", material_locations_srv.response.storage_units[0].unit_id.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_interface");
  ros::NodeHandle n;
  
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  std_srvs::Trigger begin_comp;
  
  material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  
  orders.clear();
  ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, ordersCallback);
  
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
  
  ros::spin();

  return 0;
}

