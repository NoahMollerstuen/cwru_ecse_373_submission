#include <iostream> 
#include <math.h>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/AGVControl.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "sensor_msgs/JointState.h"
#include "ur_kinematics/ur_kin.h"
#include "trajectory_msgs/JointTrajectory.h"

// Transformation header files
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Action server
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

// Vector to keep track of received orders
std::vector<osrf_gear::Order::ConstPtr> orders;

// Sevice client to query the location of specific materials
ros::ServiceClient material_locations_client;
osrf_gear::GetMaterialLocations material_locations_srv;

// Serivce client to command the gripper
ros::ServiceClient gripper_client;
osrf_gear::VacuumGripperControl gripper_control_srv;

// Service client to submit orders
ros::ServiceClient submit_order_client_1;
ros::ServiceClient submit_order_client_2;
osrf_gear::AGVControl submit_order_srv;

// Object to keep track of the latest joint states
sensor_msgs::JointState joint_states;

// Object to keep track of the gripper state
osrf_gear::VacuumGripperState gripper_state;

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

// Declare the transformation buffer to maintain a list of transformations
tf2_ros::Buffer tfBuffer;

double T_pose[4][4], T_des[4][4];
double q_pose[6], q_sols[8][6];
int msg_count = 0;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectories_as;

trajectory_msgs::JointTrajectory setupJointTrajectory(trajectory_msgs::JointTrajectory joint_trajectory) {
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	joint_trajectory.header.seq = msg_count++;
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
	// Set the names of the joints being used.  All must be present.
	joint_trajectory.joint_names.clear();
	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");

	// Set a start and end point.
	joint_trajectory.points.resize(2);

	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.01);

	// Setup the start and end points lists
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

	return joint_trajectory;
}

void callJointTrajectoriesServer(trajectory_msgs::JointTrajectory joint_trajectory) {
	// Create the structure to populate for running the Action Server.
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
	
	// It is possible to reuse the JointTrajectory from above
	joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
	
	// The header and goal (not the tolerances) of the action must be filled out as well.
	// (rosmsg show control_msgs/FollowJointTrajectoryAction)
	joint_trajectory_as.action_goal.header.seq = msg_count;
	joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.header.frame_id = "/world";

	joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.goal_id.id = std::to_string(msg_count);

	actionlib::SimpleClientGoalState state = trajectories_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
	ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
}

void moveArmToPosition(float x, float y, float z, ros::Duration duration = ros::Duration(1.0)) {
	// Where is the end effector given the joint angles.
	// joint_states.position[0] is the linear_arm_actuator_joint
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];

	ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

	T_des[0][3] = (double)x;
	T_des[1][3] = (double)y;
	T_des[2][3] = (double)z;
	T_des[3][3] = 1.0;
	// The orientation of the end effector so that the end effector is down.
	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0;  T_des[3][1] = 0.0; T_des[3][2] = 0.0;

	int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sols);

	trajectory_msgs::JointTrajectory joint_trajectory;

	joint_trajectory = setupJointTrajectory(joint_trajectory);
	
	// Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}

	int best_solution_index = -1;
	for (int i = 0; i < num_sols; i++) {
		double shoulder_angle = q_sols[i][1];
		double wrist_1_angle = q_sols[i][3];

		// Ignore solutions where the shoulder or wrist are pointed backwards
		if (
			abs(M_PI - wrist_1_angle) <= M_PI / 2 &&
			shoulder_angle >= 4 * M_PI / 3
			) {
			best_solution_index = i;
			break;
		}
	}

	if (best_solution_index == -1) {
		ROS_ERROR("Could not find an IK solution for an end effector poition of %f, %f, %f", x, y, z);
		return;
	}
	
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_sols[best_solution_index][indy];
	}

	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = duration;

	ROS_INFO("Moving arm to %f %f %f", x, y, z);

	callJointTrajectoriesServer(joint_trajectory);
}

void moveBaseToPosition(float goal_pos, ros::Duration duration = ros::Duration(2.0)) {
	trajectory_msgs::JointTrajectory joint_trajectory;

	joint_trajectory = setupJointTrajectory(joint_trajectory);

	// Set all joints to their current position
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}

	// Set the linear_arm_actuator_joint to the goal position.
	joint_trajectory.points[1].positions[0] = goal_pos;

	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = duration;

	ROS_INFO("Moving base to %f", goal_pos);

	callJointTrajectoriesServer(joint_trajectory);
}

void moveArmHome() {
	moveArmToPosition(-0.4, 0.1, 0.1);
}

void grabOrReleasePart(float x, float y, float z, bool grab) {
	bool succeeded = false;
	moveArmToPosition(x, y, z + 0.15);

	while(!(gripper_state.attached == grab) && ros::ok()) {	
		ros::Duration(0.5).sleep();
		moveArmToPosition(x, y, z + 0.015);
		ros::Duration(1.0).sleep();
		gripper_control_srv.request.enable = grab;
		succeeded = gripper_client.call(gripper_control_srv);
		ros::Duration(0.5).sleep();
		moveArmToPosition(x, y, z + 0.15);
	}
}

void ordersCallback(const osrf_gear::Order::ConstPtr& msg) {
	ROS_INFO("Order received: [%s]", msg->order_id.c_str());
	orders.push_back(msg);
}

void logicalCameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
	logical_camera_images[index] = msg;
	image_recieved[index] = true;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	joint_states = *msg;
}

void gripperStateCallback(const osrf_gear::VacuumGripperStateConstPtr& msg) {
	gripper_state = *msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ariac_interface");
	ros::NodeHandle n;

	// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
	tf2_ros::TransformListener tfListener(tfBuffer);
	
	// Wait for the competition to be ready
	ROS_INFO("Waiting for competition...");

	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	begin_client.waitForExistence();
	
	// Initialize service client for finding materials
	material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

	// Initialize service client for controlling the gripper
	gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");

	// Initialize service clients for controlling the agvs
	submit_order_client_1 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
	submit_order_client_2 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");

	// Subscribe to logical camera messages
	std::vector<ros::Subscriber> camera_subs;
	camera_subs.clear();
	for(int i = 0; i < 10; i++) {
		camera_subs.push_back(
			n.subscribe<osrf_gear::LogicalCameraImage>(
				camera_topics[i], 1000, [=](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void> > > msg) {
					logicalCameraCallback(i, msg);
					}
				)
			);
		image_recieved[i] = false;
	}
	
	ros::Subscriber joint_states_sub = n.subscribe("/ariac/arm1/joint_states", 1000, jointStatesCallback);

	ros::Subscriber gripper_state_sub = n.subscribe("/ariac/arm1/gripper/state", 1000, gripperStateCallback);

	// Initialize the action server
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>as("ariac/arm1/arm/follow_joint_trajectory", true);
	trajectories_as = &as;

	// Wait until a message has been recieved from every logical camera
	ROS_INFO("Waiting for logical camera messages...");
	
	int sum;
	do {
		ros::spinOnce();
		sum = 0;
		for(int i = 0; i < 10; i++) {
			sum += image_recieved[i];
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
	
	// Create a asynchronous spinner
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	ros::Rate loop_rate(10);
	while(ros::ok()) {
		if (orders.size() > 0) {
			// Pop an order from the front of the order queue
			osrf_gear::Order::ConstPtr order = orders[0];
			orders.erase(orders.begin());
			ROS_INFO("Processing order [%s]", order->order_id.c_str());
			
			for (osrf_gear::Shipment shipment : order->shipments) {
				bool agv1 = shipment.agv_id == "agv1";

				for (osrf_gear::Product product : shipment.products) {
					std::string part_type = product.type;
					ROS_INFO("Product type: [%s]", part_type.c_str());
					
					// Query the location of the desired type of part
					material_locations_srv.request.material_type = part_type;
					int call_succeeded;
					call_succeeded = material_locations_client.call(material_locations_srv);
					
					if(!call_succeeded) {
						ROS_ERROR("Call to /ariac/material_locations failed!");
						continue;
					}
					std::string bin;
					std::string unit_id = material_locations_srv.response.storage_units[0].unit_id;
					if(unit_id == "belt") {
						bin = material_locations_srv.response.storage_units.back().unit_id;
					} else {
						bin = unit_id;
					}
					ROS_INFO("This product can be found in bin [%s]", bin.c_str());
					
					bool done = false;

					// Search the logical camera images for the part
					for(int i = 0; i < 10; i++) {
						if(camera_topics[i].find(bin) != std::string::npos) {
							// This camera topic matches the name of the bin
							for(osrf_gear::Model model : logical_camera_images[i]->models) {
								if(model.type == part_type) {
									ROS_INFO("Model type: %s", model.type.c_str());
									ROS_INFO("Bin number: %s", bin.c_str());
									ROS_INFO("Part located at x=%f, y=%f, z=%f", model.pose.position.x, model.pose.position.y, model.pose.position.z);
									
									double base_pos;
									if (bin == "bin4") {
										base_pos = -0.2;
									} else if (bin == "bin5") {
										base_pos = 0.5;
									} else if (bin == "bin6") {
										base_pos = 1.5;
									}

									moveBaseToPosition(base_pos);
									ros::Duration(4.0).sleep();

									// Retrieve the transformation
									geometry_msgs::TransformStamped tfStamped;
									try {
										tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + bin + "_frame", ros::Time(0.0), ros::Duration(1.0));
									} catch (tf2::TransformException &ex) {
										ROS_ERROR("%s", ex.what());
									}
									
									// Create variables
									geometry_msgs::PoseStamped part_pose, goal_pose;

									// Copy pose from the logical camera.
									part_pose.pose = model.pose;
									tf2::doTransform(part_pose, goal_pose, tfStamped);
									
									// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
									goal_pose.pose.orientation.w = 0.707;
									goal_pose.pose.orientation.x = 0.0;
									goal_pose.pose.orientation.y = 0.707;
									goal_pose.pose.orientation.z = 0.0;
									
									// Pickup part
									grabOrReleasePart(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, true);
									ros::Duration(0.5).sleep();
									moveArmHome();
									ros::Duration(1.0).sleep();

									if (agv1) {
										moveBaseToPosition(2.3);
									} else {
										moveBaseToPosition(-2.3);
									}
									ros::Duration(4.5).sleep();
									

									// Retrieve the transformation
									try {
										tfStamped = tfBuffer.lookupTransform("arm1_base_link", agv1 ? "kit_tray_1" : "kit_tray_2", ros::Time(0.0), ros::Duration(1.0));
									} catch (tf2::TransformException &ex) {
										ROS_ERROR("%s", ex.what());
									}

									geometry_msgs::PoseStamped target_pose, agv_goal_pose;

									target_pose.pose = product.pose;
									tf2::doTransform(target_pose, agv_goal_pose, tfStamped);

									grabOrReleasePart(agv_goal_pose.pose.position.x, agv_goal_pose.pose.position.y, agv_goal_pose.pose.position.z, false);
									ros::Duration(1.0).sleep();
									moveArmHome();
									ros::Duration(1.0).sleep();

									done = true;
									break;
								}
							}
						}
						if (done) break;
					}
					ROS_INFO("Finished product");
				}

				bool call_succeeded;
				submit_order_srv.request.shipment_type = shipment.shipment_type;
				if (agv1) {
					call_succeeded = submit_order_client_1.call(submit_order_srv);
				} else {
					call_succeeded = submit_order_client_2.call(submit_order_srv);
				}
				
				if (call_succeeded) {
					if (call_succeeded)
					ROS_WARN("Shipment %s with message \"%s\"", submit_order_srv.response.success ? "succeeded" : "failed", submit_order_srv.response.message.c_str());
				} else {
					ROS_ERROR("Failed to call service to notify agv");
				}
			}
		}
		loop_rate.sleep();
	}

	return 0;
}