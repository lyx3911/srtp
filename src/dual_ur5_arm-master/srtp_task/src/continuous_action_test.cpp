// continuous_action_test
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


static const std::string PLANNINH_GROUP = "dual_arms";

class dual_arm
{
    public:
        dual_arm(ros::NodeHandle &n);
        ~dual_arm(){};
        ros::NodeHandle& n;

        //当前状态
        const robot_state::JointModelGroup* joint_model_group;             
        moveit::core::RobotStatePtr current_state;

        //plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        void init();
        void grasp(double close_len, moveit::planning_interface::MoveGroupInterface &grasp_group);
        std::vector<double> gripper_open_config(double open_len);
        std::vector <double> get_armjoint_values(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &arm_group);
        void setJointValues(std::vector<double> values, moveit::planning_interface::MoveGroupInterface &group);
        bool perform_action(double left_x, double left_y, double left_z, double left_roll, double left_pitch, double left_yaw,
                            double right_x, double right_y, double right_z, double right_roll, double right_pitch, double right_yaw,
                            double open_left, double open_right);

    private:
        //group的每一个关节角大小和名称
        std::vector <double> joint_values;
        std::vector <std::string> joint_names;
        std::map <std::string, int> joints_list; //关节名，编号，方便后面查表

        moveit::planning_interface::MoveGroupInterface group = moveit::planning_interface::MoveGroupInterface(PLANNINH_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        //四个部分都是group的subgroup
        moveit::planning_interface::MoveGroupInterface left_arm = moveit::planning_interface::MoveGroupInterface("left_arm");
        moveit::planning_interface::MoveGroupInterface right_arm = moveit::planning_interface::MoveGroupInterface("right_arm");	
        moveit::planning_interface::MoveGroupInterface left_gripper = moveit::planning_interface::MoveGroupInterface("left_gripper");
        moveit::planning_interface::MoveGroupInterface right_gripper = moveit::planning_interface::MoveGroupInterface("right_gripper");
};


dual_arm::dual_arm(ros::NodeHandle& n):
    n(n)
{
    this->joint_model_group = this->group.getCurrentState()->getJointModelGroup("dual_arms");
    this->current_state = this->group.getCurrentState();
}

void dual_arm::init()
{
    //init 初始化，得到关节角的名称和初始的关节角大小
	this->joint_names = this->group.getJointNames();
	(this->current_state)->copyJointGroupPositions(this->joint_model_group, this->joint_values);
	for(int i=0;i<this->joint_names.size();i++){
		std::cout << this->joint_names[i] << ": " << this->joint_values[i] << std::endl;
		this->joints_list[this->joint_names[i]] = i;
	}
}

//爪夹张开的大小转换成关节角
//open max: 0.1
//open min: 0.0061
std::vector<double> dual_arm::gripper_open_config(double open_len)
{
	double open_max = 0.1;
	double open_min = 0.0061;
	if(open_len > open_max) open_len = open_max;
	if(open_len < open_min) open_len = open_min;
	double open_len_off = open_len - 0.016 + 0.0099;
	double alpha = std::asin(open_len_off/2/0.057);
	double theta = std::atan(0.0493634/0.0285) - alpha;

	std::vector<double> angle{theta, theta/1.1, theta, theta/1.1};
    // for(double a: angle) std::cout << a << "  " ;
    // std::cout << std::endl;

	return angle;
}

//输入：末端的目标位姿和需要规划的arm_group,(left_arm或right_arm)
//输出：各个关节的目标值
//不直接用group来规划的原因，不是只由chain构成的机械臂，规划失败率太高
std::vector <double> dual_arm::get_armjoint_values(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &arm_group)
{
	// std::cout << "get armjoint values" <<std::endl;
	arm_group.setPoseTarget(target_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = false;
	while(!success)
		success = (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
	int size = trajectory.joint_trajectory.points.size();
	// std::cout << trajectory.joint_trajectory.points[size-1].positions[0] << std::endl;
	// std::cout << trajectory.joint_trajectory.points[size-1].positions.size() << std::endl;
	return trajectory.joint_trajectory.points[size-1].positions;
}

//设置dual_arm的目标关节角
void dual_arm::setJointValues(std::vector<double> values, moveit::planning_interface::MoveGroupInterface &group)
{
	// std::cout << "set joint values" << std::endl;
	std::vector <std::string> names = group.getJointNames();
	for(int i=0;i<values.size();i++){
		joint_values[joints_list[names[i]]] = values[i];
	}
}

//单个爪夹抓取
void dual_arm::grasp(double close_len, moveit::planning_interface::MoveGroupInterface &grasp_group)
{
	std::vector <double> gripper_list = gripper_open_config(close_len);

	grasp_group.setJointValueTarget(gripper_list);

	moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
	
	bool success = (grasp_group.plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");

	// grasp_group.execute(grasp_plan);


	moveit_msgs::RobotTrajectory trajectory = grasp_plan.trajectory_;
	
	moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
	robot_trajectory::RobotTrajectory rt(grasp_group.getCurrentState()->getRobotModel(),grasp_group.getName());
	rt.setRobotTrajectoryMsg(*grasp_group.getCurrentState(),trajectory);
	trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
	iptp.computeTimeStamps(rt,1,1);

	rt.getRobotTrajectoryMsg(joinedPlan.trajectory_);

	if(!grasp_group.execute(joinedPlan))
	{
		ROS_ERROR("Failed to execute plan");
		return;
	}

}


//执行某个动作
//输入：左手，右手的末端目标位姿(rpy欧拉角表示)，两个爪夹的张开大小
//成功执行返回true，失败返回false
bool dual_arm::perform_action(double left_x, double left_y, double left_z, double left_roll, double left_pitch, double left_yaw,
    double right_x, double right_y, double right_z, double right_roll, double right_pitch, double right_yaw,
    double open_left, double open_right)
{
    //left_arm
    geometry_msgs::Pose target_pose_left;
    target_pose_left.orientation = tf::createQuaternionMsgFromRollPitchYaw(left_roll,left_pitch,left_yaw);
    target_pose_left.position.x = left_x;
    target_pose_left.position.y = left_y;
    target_pose_left.position.z = left_z;
    std::vector<double>left_joints = get_armjoint_values(target_pose_left,left_arm);
    setJointValues(left_joints, left_arm);

    //right_arm
    geometry_msgs::Pose target_pose_right;
    target_pose_right.orientation = tf::createQuaternionMsgFromRollPitchYaw(right_roll, right_pitch, right_yaw);
    target_pose_right.position.x = right_x;
    target_pose_right.position.y = right_y;
    target_pose_right.position.z = right_z;
    std::vector<double>right_joints = get_armjoint_values(target_pose_right, right_arm);
    setJointValues(right_joints, right_arm);

    //left_gripper
    std::vector<double> left_gripper_joints = gripper_open_config(open_left);
    setJointValues(left_gripper_joints, left_gripper);

    //right_gripper
    std::vector<double> right_gripper_joints = gripper_open_config(open_right);
    setJointValues(right_gripper_joints, right_gripper);

    //plan
    group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success){
        ROS_INFO("plan  %s",success?"SUCCESS":"FAILD");
        return success;
    }

    group.execute(plan);  
    return true;
}
    


int main(int argc, char **argv)
{
	ros::init(argc,argv,"continuous_action_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

    dual_arm dual_arm_(node_handle);

///////////////// init ////////////////////////////////////
	dual_arm_.init();

///////////////////////////// 动作测试 /////////////////////////////////

	dual_arm_.perform_action(0.28, 0.4, 0.2, 0, M_PI/2, 0, 0.28, -0.4, 0.2, 0, M_PI/2, 0, 0.1, 0.1);

    dual_arm_.perform_action(0.28, 0.4, 0.2, 0, M_PI/2, 0, 0.28, -0.4, 0.2, 0, M_PI/2, 0, 0.01, 0.01);

    dual_arm_.perform_action(0.50, 0.4, 0.2, 0, M_PI/2, 0, 0.50, -0.4, 0.2, 0, M_PI/2, 0, 0.01, 0.01);

    dual_arm_.perform_action(0.50, 0.4, 0.2, 0, M_PI/2, 0, 0.50, -0.4, 0.2, 0, M_PI/2, 0, 0.1, 0.1);

	ros::shutdown();
	return 0;
}