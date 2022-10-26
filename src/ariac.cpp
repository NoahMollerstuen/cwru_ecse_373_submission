#include <iostream> 

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"

// Vector to keep track of received orders
std::vector<osrf_gear::Order::ConstPtr> orders;

// Sevice client to query the location of specific materials
ros::ServiceClient material_locations_client;
osrf_gear::GetMaterialLocations material_locations_srv;

// Array of logical camera topics
std::string camera_topics[] = {
	"/ariac/logical_camera_bin1",
	"/ariac/logical_camera_bin2",
	"/ariac/logical_camera_bin3",
	"/ariac/logical_camera_bin4",
	"/ariac/logical_camera_bin5",
	"/ariac/logical_camera_bin6",
	"/ariac/logical_camera_agv1",
	"/ariac/logical_camera_agv2",
	"/ariac/quality_control_sensor_1",
	"/ariac/quality_control_sensor_2"
};

// Array to keep track of logical camera images
osrf_gear::LogicalCameraImage::ConstPtr logical_camera_images[10];
int image_recieved[10];


void ordersCallback(const osrf_gear::Order::ConstPtr& msg) {
	ROS_INFO("Order received: [%s]", msg->order_id.c_str());
	orders.push_back(msg);
	std::string part_type = msg->shipments[0].products[0].type;
	ROS_INFO("First product type: [%s]", part_type.c_str());
	
	// Query the location of the desired type of part
	material_locations_srv.request.material_type = part_type;
	int call_succeeded;
	call_succeeded = material_locations_client.call(material_locations_srv);
	
	if(!call_succeeded) {
		ROS_ERROR("Call to /ariac/material_locations failed!");
		return;
	}
  
	std::string bin = material_locations_srv.response.storage_units[0].unit_id;
	ROS_INFO("This product can be found in bin [%s]", bin.c_str());
	
	// Search the logical camera images for the part
	for(int i = 0; i < 10; i++) {
		if(camera_topics[i].find(bin) != std::string::npos) {
			// This camera topic matches the name of the bin
			for(osrf_gear::Model model : logical_camera_images[i]->models) {
				if(model.type == part_type) {
					ROS_WARN("Part located at x=%f, y=%f, z=%f", model.pose.position.x, model.pose.position.y, model.pose.position.z);
					break;
				}
			}
			break;
		}
	}
}

void logicalCameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
	logical_camera_images[index] = msg;
	image_recieved[index] = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ariac_interface");
	ros::NodeHandle n;
	
	// Wait for the competition to be ready
	ROS_INFO("Waiting for competition...");
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	begin_client.waitForExistence();
	
	// Create service client for finding materials
	material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	
	// Subscribe to logical camera messages
	std::vector<ros::Subscriber> camera_subs;
	camera_subs.clear();
	for(int i = 0; i < 10; i++) {
		camera_subs.push_back(n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics[i], 1000, [=](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void> > > msg) { logicalCameraCallback(i, msg); }));
		image_recieved[i] = false;
	}
	
	// Wait until a message has been recieved from every logical camera
	ROS_INFO("Waiting for logical camera messages...");
	int sum;
	do {
		ros::spinOnce();
		sum = 0;
		for(int i = 0; i < 10; i++) {
			sum += image_recieved[i];
			//ROS_INFO("%d", image_recieved[i]);
		}
	} while (sum < 10);
	
	// Subscribe to incoming orders messages
	orders.clear();
	ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, ordersCallback);
  
	// Start the competition
	std_srvs::Trigger begin_comp;
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

