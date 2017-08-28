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
	trajectory.joint_names.resize(1);
	trajectory.joint_names[0] = "joint1";

	// positions and velocities field
	trajectory_points.positions.resize(1);

	double fraction_of_range;
	int time_5 = 5;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_5+1; i++) { // there are time_5+1 points, including start and end
		fraction_of_range = (double)i/time_5;
		for (int j=0; j<1; j++) { // there are 1 joints
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
	trajectory.joint_names.resize(1);
	trajectory.joint_names[0] = "joint1";


	// positions and velocities field
	trajectory_points.positions.resize(1);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;


	double time_delay = 5.0; // delay between every task


	///////////////////////////////////////
	// 0.move to the original pose
	///////////////////////////////////////

	ROS_INFO("step 0: move to initial position.");

	// get the original joint positions when this node is invoked
	std::vector<double> origin_jnts;
	origin_jnts.resize(1);
	for (int i=0; i<1; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}
	// assign current joints to start joints


	// define the safe point, avoid singularity at origin
	std::vector<double> safe_jnts;
	safe_jnts.resize(1);
	safe_jnts[0] = 0; // joint1, at its origin

	// assign the safe joints to end joints

	move_arm(origin_jnts, safe_jnts);

	ROS_INFO("step 0 is done.");

	// if here, task 1 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	
	//////////////////////////////////////
	// 1.rotate 90 degree
	//////////////////////////////////////

	ROS_INFO("step 1: rotate 90 degree");

	// assign the start joints and end joints


	std::vector<double> safe1_jnts;
	safe1_jnts.resize(1);
	safe1_jnts[0] = M_PI/2; // joint1, at its origin


	move_arm(safe_jnts, safe1_jnts);

	ROS_INFO("step 1 is done.");

	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	//////////////////////////////////////
	// 2.move back
	//////////////////////////////////////

	ROS_INFO("step 2: move back");


	move_arm(safe1_jnts, safe_jnts);

	ROS_INFO("step 2 is done.");

	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	ROS_INFO(" task is finished! Thank you for watching");

	return 0;
}

