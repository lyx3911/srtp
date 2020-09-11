#ifndef DUAL_ARM_H
#define DUAL_ARM_H

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

        //规划组
        moveit::planning_interface::MoveGroupInterface group = moveit::planning_interface::MoveGroupInterface(PLANNINH_GROUP);
        
        //四个部分都是group的subgroup
        moveit::planning_interface::MoveGroupInterface left_arm = moveit::planning_interface::MoveGroupInterface("left_arm");
        moveit::planning_interface::MoveGroupInterface right_arm = moveit::planning_interface::MoveGroupInterface("right_arm");	
        moveit::planning_interface::MoveGroupInterface left_gripper = moveit::planning_interface::MoveGroupInterface("left_gripper");
        moveit::planning_interface::MoveGroupInterface right_gripper = moveit::planning_interface::MoveGroupInterface("right_gripper");

        //plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        //初始化
        void init();

        //抓取动作，仅计算爪夹，不需要arm移动时比较快
        void grasp(double close_len, moveit::planning_interface::MoveGroupInterface &grasp_group); //有问题,勿用 用下一个
        // 重载
        void grasp(double left_open, double right_open);

        std::vector<double> gripper_open_config(double open_len);
        std::vector <double> get_armjoint_values(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &arm_group);
        void setJointValues(std::vector<double> values, moveit::planning_interface::MoveGroupInterface &group);
        
        //同时计算gripper和arm的关节角，规划比较费时间
        bool perform_action(double left_x, double left_y, double left_z, double left_roll, double left_pitch, double left_yaw,
                            double right_x, double right_y, double right_z, double right_roll, double right_pitch, double right_yaw,
                            double open_left, double open_right);


    private:
        //group的每一个关节角大小和名称
        std::vector <double> joint_values;
        std::vector <std::string> joint_names;
        std::map <std::string, int> joints_list; //关节名，编号，方便后面查表

};


#endif