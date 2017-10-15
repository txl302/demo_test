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
	trajectory.joint_names.resize(7);
	trajectory.joint_names[0] = "joint1";
	trajectory.joint_names[1] = "joint2";
	trajectory.joint_names[2] = "joint3";
	trajectory.joint_names[3] = "joint4";
	trajectory.joint_names[4] = "joint5";
	trajectory.joint_names[5] = "joint6";
	trajectory.joint_names[6] = "joint7";

	// positions and velocities field
	trajectory_points.positions.resize(7);

	double fraction_of_range;
	int time_5 = 1;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_5+1; i++) { // there are time_5+1 points, including start and end
		fraction_of_range = (double)i/time_5;
		for (int j=0; j<7; j++) { // there are 5 joints
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
	trajectory.joint_names.resize(7);
	trajectory.joint_names[0] = "joint1";
	trajectory.joint_names[1] = "joint2";
	trajectory.joint_names[2] = "joint3";
	trajectory.joint_names[3] = "joint4";
	trajectory.joint_names[4] = "joint5";
	trajectory.joint_names[5] = "joint6";
	trajectory.joint_names[6] = "joint7";

	// positions and velocities field
	trajectory_points.positions.resize(7);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;


	double time_delay = 5; // delay between every task

	std::vector<double> safe_jnts;
	std::vector<double> front_jnts;
	std::vector<double> top_jnts;
	std::vector<double> side_jnts;
	std::vector<double> rotate_jnts;
	std::vector<double> back_jnts;

	
	safe_jnts.resize(7);
	front_jnts.resize(7);
	top_jnts.resize(7);
	side_jnts.resize(7);
	rotate_jnts.resize(7);
	back_jnts.resize(7);





	//safe pose
	safe_jnts[0] = 0; // joint1, at its origin
	safe_jnts[1] = M_PI/4; // joint2, a little bit forward
	safe_jnts[2] = -safe_jnts[1]; // joint3, a little bit forward
	safe_jnts[3] = 0; // joint4, parallel to the ground
	safe_jnts[4] = 0;
	safe_jnts[5] = 0;
	safe_jnts[6] = 0;

	//pose 1
	front_jnts[0] = 0; // joint1, at its origin
	front_jnts[1] = M_PI/6; // joint2, a little bit forward
	front_jnts[2] = -front_jnts[1]; // joint3, a little bit forward
	front_jnts[3] = 0; // joint4, parallel to the ground
	front_jnts[4] = 0;
	front_jnts[5] = 0;
	front_jnts[6] = 0;


	//pos2
	top_jnts[0] = 0; // joint1, at its origin
	top_jnts[1] = -M_PI/8; // joint2, a little bit forward
	top_jnts[2] = M_PI/2; // joint3, a little bit forward
	top_jnts[3] = 0; // joint4, parallel to the ground
	top_jnts[4] = M_PI/2;
	top_jnts[5] = 0;
	top_jnts[6] = 0;

	//pos3
	side_jnts[0] = M_PI/4; // joint1, at its origin
	side_jnts[1] = -M_PI/4; // joint2, a little bit forward
	side_jnts[2] = M_PI/2; // joint3, a little bit forward
	side_jnts[3] = -M_PI/2; // joint4, parallel to the ground
	side_jnts[4] = 0;
	side_jnts[5] = 0;
	side_jnts[6] = 0;

	//pos4
	rotate_jnts[0] = side_jnts[0]; // joint1, at its origin
	rotate_jnts[1] = side_jnts[1]; // joint2, a little bit forward
	rotate_jnts[2] = side_jnts[2]; // joint3, a little bit forward
	rotate_jnts[3] = side_jnts[3]; // joint4, parallel to the ground
	rotate_jnts[4] = side_jnts[4];
	rotate_jnts[5] = side_jnts[5];
	rotate_jnts[6] = M_PI;

	//safe pose 0
	back_jnts[0] = front_jnts[0]; // joint1, at its origin
	back_jnts[1] = front_jnts[1]; // joint2, a little bit forward
	back_jnts[2] = front_jnts[2]; // joint3, a little bit forward
	back_jnts[3] = front_jnts[3]; // joint4, parallel to the ground
	back_jnts[4] = front_jnts[4];
	back_jnts[5] = front_jnts[5];
	back_jnts[6] = rotate_jnts[6];


	ROS_INFO("step 0: move to initial position.");

	// get the original joint positions when this node is invoked
	std::vector<double> origin_jnts;
	origin_jnts.resize(7);
	for (int i=0; i<5; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}

	move_arm(origin_jnts, safe_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	std::cin.get();
	
	ROS_INFO("step 1: move to right front of the object");
	move_arm(safe_jnts, front_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	std::cin.get();

	ROS_INFO("step 2: move to top of the object");
	move_arm(front_jnts, top_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	std::cin.get();

	ROS_INFO("step 3: move to the side of the object");
	move_arm(top_jnts, side_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	std::cin.get();

	ROS_INFO("step 4: rotate the platform");
	move_arm(side_jnts, rotate_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	std::cin.get();

	ROS_INFO("step 5: move back to front of object");
	move_arm(rotate_jnts, back_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	ROS_INFO("step 6: move back to the safe position.");
	move_arm(back_jnts, safe_jnts);
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	ROS_INFO("Task is finished! Thank you for watching");

	return 0;
}

