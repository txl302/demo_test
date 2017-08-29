#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <demo_test/trajAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>

// there are repeated code in calculating the goal message, may use a function instead

// inverse kinematics function of the gripper robot

// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const demo_test::trajResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}


void move_arm(std::vector<double> start_jnts, 
				std::vector<double> end_jnts)
{
	actionlib::SimpleActionClient<demo_test::trajAction> action_client("gripper_robot_trajectory_action", true);

	demo_test::trajGoal goal;

	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;
	// joint_names field
	trajectory.joint_names.resize(5);
	trajectory.joint_names[0] = "joint1";
	trajectory.joint_names[1] = "joint2";
	trajectory.joint_names[2] = "joint3";
	trajectory.joint_names[3] = "joint4";
	trajectory.joint_names[4] = "joint5";

	// positions and velocities field
	trajectory_points.positions.resize(5);

	double fraction_of_range;
	int time_5 = 5;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_5+1; i++) { // there are time_5+1 points, including start and end
		fraction_of_range = (double)i/time_5;
		for (int j=0; j<5; j++) { // there are 5 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "gripper_robot_trajectory_action_client_node");
	ros::NodeHandle nh;

	// initialize an action client
	actionlib::SimpleActionClient<demo_test::trajAction> action_client(
		"gripper_robot_trajectory_action", true);
	// try to connect the client to action server
	bool server_exist = action_client.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server; retrying");
		bool server_exist = action_client.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	// if here, then connected to the server
	ROS_INFO("connected to action server");

	demo_test::trajGoal goal;
	// instantiate goal message
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;
	// joint_names field
	trajectory.joint_names.resize(5);
	trajectory.joint_names[0] = "joint1";
	trajectory.joint_names[1] = "joint2";
	trajectory.joint_names[2] = "joint3";
	trajectory.joint_names[3] = "joint4";
	trajectory.joint_names[4] = "joint5";

	// positions and velocities field
	trajectory_points.positions.resize(5);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;


	double time_delay = 10.0; // delay between every task


	///////////////////////////////////////
	// 0.move the gripper to the safe point
	///////////////////////////////////////

	ROS_INFO("step 0: move to initial position.");

	// get the original joint positions when this node is invoked
	std::vector<double> origin_jnts;
	origin_jnts.resize(5);
	for (int i=0; i<5; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}
	// assign current joints to start joints


	// define the safe point, avoid singularity at origin
	std::vector<double> safe_jnts;
	safe_jnts.resize(6);
	safe_jnts[0] = 0; // joint1, at its origin
	safe_jnts[1] = 150/180.0*M_PI; // joint2, a little bit forward
	safe_jnts[2] = -60/180.0*M_PI; // joint3, a little bit forward
	safe_jnts[3] = 0; // joint4, parallel to the ground
	safe_jnts[4] = 0;

	// assign the safe joints to end joints

	move_arm(origin_jnts, safe_jnts);

	ROS_INFO("step 0 is done.");

	// if here, task 1 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	
	//////////////////////////////////////
	// 1.move to front surface
	//////////////////////////////////////

	ROS_INFO("step 1: move to the front surface");

	// assign the start joints and end joints


	std::vector<double> safe1_jnts;
	safe1_jnts.resize(6);
	safe1_jnts[0] = M_PI/2; // joint1, at its origin
	safe1_jnts[1] = 150/180.0*M_PI; // joint2, a little bit forward
	safe1_jnts[2] = -60/180.0*M_PI; // joint3, a little bit forward
	safe1_jnts[3] = 0; // joint4, parallel to the ground
	safe1_jnts[4] = 0;


	move_arm(safe_jnts, safe1_jnts);

	ROS_INFO("step 1 is done.");

	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	//////////////////////////////////////
	// 2.move to the side surface
	//////////////////////////////////////

	ROS_INFO("step 2: move to the side surface");



	std::vector<double> safe2_jnts;
	safe2_jnts.resize(6);
	safe2_jnts[0] = M_PI/4; // joint1, at its origin
	safe2_jnts[1] = 120.0/180.0*M_PI; // joint2, a little bit forward
	safe2_jnts[2] = -30/180.0*M_PI; // joint3, a little bit forward
	safe2_jnts[3] = safe2_jnts[0] - M_PI; // joint4, parallel to the ground
	safe2_jnts[4] = 0;


	move_arm(safe1_jnts, safe2_jnts);

	ROS_INFO("step 2 is done.");


	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	//////////////////////////////////////
	// 3.move to the top surface
	//////////////////////////////////////

	ROS_INFO("step 3: move to the top surface");



	std::vector<double> safe3_jnts;
	safe3_jnts.resize(6);
	safe3_jnts[0] = M_PI/2; // joint1, at its origin
	safe3_jnts[1] = 80.0/180.0*M_PI; // joint2, a little bit forward
	safe3_jnts[2] = M_PI - safe3_jnts[1]; // joint3, a little bit forward
	safe3_jnts[3] = 0; // joint4, parallel to the ground
	safe3_jnts[4] = 0;


	move_arm(safe2_jnts, safe3_jnts);

	ROS_INFO("step 3 is done.");

	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	//////////////////////////////////////
	// 4.move to the side surface
	//////////////////////////////////////

	ROS_INFO("step 4: move to the side surface");



	std::vector<double> safe4_jnts;
	safe4_jnts.resize(6);
	safe4_jnts[0] = M_PI/4; // joint1, at its origin
	safe4_jnts[1] = 120.0/180.0*M_PI; // joint2, a little bit forward
	safe4_jnts[2] = -30/180.0*M_PI; // joint3, a little bit forward
	safe4_jnts[3] = safe4_jnts[0] - M_PI; // joint4, parallel to the ground
	safe4_jnts[4] = M_PI/2;


	move_arm(safe3_jnts, safe4_jnts);

	ROS_INFO("step 4 is done.");


	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	//////////////////////////////////////
	// 5.move to the front sruface
	//////////////////////////////////////

	ROS_INFO("step 5: move to the front surface");

	std::vector<double> safe5_jnts;
	safe5_jnts.resize(6);
	safe5_jnts[0] = M_PI/2; // joint1, at its origin
	safe5_jnts[1] = 150/180.0*M_PI; // joint2, a little bit forward
	safe5_jnts[2] = -60/180.0*M_PI; // joint3, a little bit forward
	safe5_jnts[3] = 0; // joint4, parallel to the ground
	safe5_jnts[4] = M_PI/2;

	move_arm(safe4_jnts, safe5_jnts);

	ROS_INFO("step 5 is done.");


	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task



	/////////////////////////////////////////////
	// 6.move back to the safe point
	/////////////////////////////////////////////

	ROS_INFO("step 6: move back to the safe position.");

	// assign the start joints and end joints

	move_arm(safe5_jnts, safe_jnts);


	ROS_INFO("step 6 is done.");
	
	// if here, task 10 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	/////////////////////////////////////////////
	// 7.dock the robotic arm at the designated spot
	/////////////////////////////////////////////

	ROS_INFO("step 7: move back to the original position.");

	std::vector<double> origin0_jnts;
	origin0_jnts.resize(6);
	origin0_jnts[0] = 0; // joint1, at its origin
	origin0_jnts[1] = 40.0/180.0*M_PI; // joint2, a little bit forward
	origin0_jnts[2] = 90.0/180.0*M_PI; // joint3, a little bit forward
	origin0_jnts[3] = 0; // joint4, parallel to the ground
	origin0_jnts[4] = 0;

	// assign the start joints and end joints

	move_arm(safe_jnts, origin0_jnts);


	ROS_INFO("step 7 is done.");
	
	// if here, task 10 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	ROS_INFO("Task is finished! Thank you for watching");

	return 0;
}

