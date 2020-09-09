#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <vector>
#include <time.h>
// #include <Eigen/Dense>


std::vector<double> gripper_open_config(double open_len)
{
	double open_max = 0.1;
	double open_min = 0.0061;
	if(open_len > open_max) open_len = open_max;
	if(open_len < open_min) open_len = open_min;
	double open_len_off = open_len - 0.016 + 0.0099;
	double alpha = std::asin(open_len_off/2/0.057);
	double theta = std::atan(0.0493634/0.0285) - alpha;

	std::vector<double> angle{theta, theta/1.1, theta, theta/1.1};
	return angle;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"srtp_task");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNINH_GROUP = "dual_arms";
	moveit::planning_interface::MoveGroupInterface group(PLANNINH_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	moveit::planning_interface::MoveGroupInterface left_gripper("left_gripper");
	moveit::planning_interface::MoveGroupInterface right_gripper("right_gripper");

	const robot_state::JointModelGroup* joint_model_group =  
		group.getCurrentState()->getJointModelGroup(PLANNINH_GROUP);

	

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
	target_pose1.position.x = 0.28;
	target_pose1.position.y = 0.4;
	target_pose1.position.z = 0.2;


	group.setPoseTarget(target_pose1,"left_ee_link");
	// group.setMaxVelocityScalingFactor(0.1);

	geometry_msgs::Pose target_pose2;
	target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
	target_pose2.position.x = 0.28;
	target_pose2.position.y = -0.4;
	target_pose2.position.z = 0.4;

	group.setPoseTarget(target_pose2,"right_ee_link");
	// group.setMaxVelocityScalingFactor(0.1);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");

	sleep(3);
	if(success)
		group.execute(plan);
	sleep(3);


	//grasp
	std::vector <double> gripper_list = gripper_open_config(0.1);
	left_gripper.setJointValueTarget(gripper_list);
	right_gripper.setJointValueTarget(gripper_list);

	success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
		group.execute(plan);
	sleep(3);


	ros::shutdown();
	return 0;
}