#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include "time_optimal_trajectory_generation.h"
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <vector>
#include <time.h>
// #include <boost/thread.hpp>
// #include <unistd.h>


//group的每一个关节角大小和名称
std::vector <double> joint_values;
std::vector <std::string> joint_names;
std::map <std::string, int> joints_list; //关节名，编号，方便后面查表


//爪夹张开的大小转换成关节角
std::vector<double> gripper_open_config(double open_len)
{
	double open_max = 0.1;
	double open_min = 0.0061;
	if(open_len > open_max) open_len = open_max;
	if(open_len < open_min) open_len = open_min;
	double open_len_off = open_len - 0.016 + 0.0099;
	double alpha = std::asin(open_len_off/2/0.057);
	double theta = std::atan(0.0493634/0.0285) - alpha;

	std::vector<double> angle{theta*10, theta/1.1*10, theta*10, theta/1.1*10};
	return angle;
}

//输入：末端的目标位姿和需要规划的arm_group,(left_arm或right_arm)
//输出：各个关节的目标值
//不直接用group来规划的原因，不是只由chain构成的机械臂，规划失败率太高
std::vector <double> get_armjoint_values(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &arm_group)
{
	std::cout << "get armjoint values" <<std::endl;
	arm_group.setPoseTarget(target_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = false;
	while(!success)
		success = (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
	int size = trajectory.joint_trajectory.points.size();
	// std::cout << trajectory.joint_trajectory.points[size-1].positions[0] << std::endl;
	std::cout << trajectory.joint_trajectory.points[size-1].positions.size() << std::endl;
	return trajectory.joint_trajectory.points[size-1].positions;
}

//设置dual_arm的目标关节角
void setJointValues(std::vector<double> values, moveit::planning_interface::MoveGroupInterface &group)
{
	std::cout << "set joint values" << std::endl;
	std::vector <std::string> names = group.getJointNames();
	for(int i=0;i<values.size();i++){
		joint_values[joints_list[names[i]]] = values[i];
	}
}

//单个爪夹抓取
void grasp(double close_len, moveit::planning_interface::MoveGroupInterface &grasp_group)
{
	std::vector <double> gripper_list = gripper_open_config(close_len);

	grasp_group.setJointValueTarget(gripper_list);

	moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
	
	bool success = (grasp_group.plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");

	grasp_group.execute(grasp_plan);


	// moveit_msgs::RobotTrajectory trajectory = grasp_plan.trajectory_;
	
	// moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
	// robot_trajectory::RobotTrajectory rt(grasp_group.getCurrentState()->getRobotModel(),grasp_group.getName());
	// rt.setRobotTrajectoryMsg(*grasp_group.getCurrentState(),trajectory);
	// trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
	// iptp.computeTimeStamps(rt,1,1);

	// rt.getRobotTrajectoryMsg(joinedPlan.trajectory_);

	// if(!grasp_group.execute(joinedPlan))
	// {
	// 	ROS_ERROR("Failed to execute plan");
	// 	return;
	// }

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"dual_arm_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

///////////////// init ////////////////////////////////////
	static const std::string PLANNINH_GROUP = "dual_arms";
	moveit::planning_interface::MoveGroupInterface group(PLANNINH_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	//四个部分都是group的subgroup
	moveit::planning_interface::MoveGroupInterface left_arm("left_arm");
	moveit::planning_interface::MoveGroupInterface right_arm("right_arm");	
	moveit::planning_interface::MoveGroupInterface left_gripper("left_gripper");
	moveit::planning_interface::MoveGroupInterface right_gripper("right_gripper");

	//当前状态
	const robot_state::JointModelGroup* joint_model_group =  
		group.getCurrentState()->getJointModelGroup(PLANNINH_GROUP);
	moveit::core::RobotStatePtr current_state = group.getCurrentState();

	//plan
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	
	//init 初始化，得到关节角的名称和初始的关节角大小
	joint_names = group.getJointNames();
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(int i=0;i<joint_names.size();i++){
		std::cout << joint_names[i] << ": " << joint_values[i] << std::endl;
		joints_list[joint_names[i]] = i;
	}

///////////////////////////// 动作测试 /////////////////////////////////

	//设置左手目标位置
	std::vector <double> left_joints;
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
	target_pose1.position.x = 0.28;
	target_pose1.position.y = 0.4;
	target_pose1.position.z = 0.2;
	left_joints = get_armjoint_values(target_pose1,left_arm);
	setJointValues(left_joints, left_arm);
	
	//设置右手目标
	std::vector<double> right_joints;
	geometry_msgs::Pose target_pose2;
	target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
	target_pose2.position.x = 0.28;
	target_pose2.position.y = -0.4;
	target_pose2.position.z = 0.2;
	right_joints = get_armjoint_values(target_pose2,right_arm);
	setJointValues(right_joints, right_arm);

	//设置group
	group.setJointValueTarget(joint_values);

	bool success;
	success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");

	if(success)
		group.execute(plan);

	//抓取 
	std::vector <double> gripper_list = gripper_open_config(0.1);
	
	setJointValues(gripper_list, right_gripper);
	setJointValues(gripper_list, left_gripper);

	group.setJointValueTarget(joint_values);

	success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");
	if(success)
		group.execute(plan);
	sleep(3);


	ros::shutdown();
	return 0;
}