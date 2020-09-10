# 双臂协同示教学习

李盈萱 2020-09

3170103138@zju.edu.cn

[TOC]

## 环境：

- Ubuntu 16.0
- ros kinect
- moveit

有用到时间最优规划`time_optimal_trajectory_generation`但是ros kinect版本还没有集成这个功能，所以下载了源码编译。源码在include和src文件夹中`time_optimal_trajectory_generation.h` `time_optimal_trajectory_generation.cpp`

cmakelist中写的是相对路径，应该是不需要修改的。



## 文件：

srtp任务文件夹：`srtp_task`

学长的示例文件：`raw_togo`

机械臂模型文件夹：`ur_description`

椅子的模型文件：`srtp_task/mesh`

moveit配置文件：`dual_ur5_moveit_config`

- config文件夹下的`controller.yaml` `fake_controllers.yaml` `ros_controllers.yaml`文件有所修改，因为原来的没有添加爪夹的控制器



## planning groups:

- left_arm:
  - chain: left_base_link -> left_ee_link
- right_arm:
  - chain: right_base_link -> right_ee_link
- dual_arms
  - subgroups:
    - left_arm
    - right_arm
    - left_gripper
    - right_gripper
- left_gripper
  - joints
    - left_rh_l1-Revolute
    - left_rh_l2-Revolute
    - left_rh_r2-Revolute
    - left_rh_p12_rn-Revolute
  - links
    - left_rh_p12_rn_base
    - left_rh_p12_rn_l1
    - left_rh_p12_rn_l2
    - left_rh_p12_rn_r1
    - left_rh_p12_rn_r2
- right_gripper
  - joints
    - right_rh_l1-Revolute
    - right_rh_l2-Revolute
    - right_rh_p12_rn-Revolute
    - right_rh_r2-Revolute
  - links
    - right_rh_p12_rn_base
    - right_rh_p12_rn_l1
    - right_rh_p12_rn_l2
    - right_rh_p12_rn_r1
    - right_rh_p12_rn_r2



## 启动：

启动rviz

```bash
roslaunch dual_ur5_moveit_config demo.launch
```

查看椅子的装配模型

```bash
rosrun srtp_task stl_rviz_test
```

双臂协同动作测试

```bash
rosrun srtp_task dual_arm_test
```

双臂连续动作测试

```bash
rosrun srtp_task continuous_action_test
```



## 代码说明：

封装了类dual_arm

主要函数：

```c++
//一些参数的初始化
void init();

//爪夹张开大小转换成gripper的关节角
std::vector<double> gripper_open_config(double open_len);

//arm的末端位得到成arm的关节角
std::vector <double> get_armjoint_values(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &arm_group);

//设置dual_arm的目标关节角
void setJointValues(std::vector<double> values, moveit::planning_interface::MoveGroupInterface &group);

//给定left_arm, right_arm, left_gripper, right_gripper的状态，规划并执行
bool perform_action(double left_x, double left_y, double left_z, double left_roll, double left_pitch, double left_yaw,double right_x, double right_y, double right_z, double right_roll, double right_pitch, double right_yaw,double open_left, double open_right);
```



使用示例:

```c++
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
```





## 参考教程

https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html

moveit官方教程

https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html