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
#include <Eigen/Dense>
#include <string>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <dual_arm.h>


std::vector<moveit_msgs::CollisionObject> chair_components;


void addChairComponents(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
	std::vector<moveit_msgs::CollisionObject> chair_components;

	//chair surface
	moveit_msgs::CollisionObject chair_surface;
	chair_surface.header.frame_id = "world";
	chair_surface.id = "chair_surface";
	Eigen::Vector3d b(0.001,0.001,0.001);
	shapes::Mesh*m = shapes::createMeshFromResource("package://srtp_task/mesh/chair_surface.STL",b);
	shape_msgs::Mesh chair_surface_mesh;
	shapes::ShapeMsg chair_surface_mesh_msg;
	shapes::constructMsgFromShape(m,chair_surface_mesh_msg);
	chair_surface_mesh = boost::get<shape_msgs::Mesh>(chair_surface_mesh_msg);

	geometry_msgs::Pose chair_surface_pose;
	chair_surface_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI,0);
    chair_surface_pose.position.x =  0.4;
    chair_surface_pose.position.y =  0.0;
    chair_surface_pose.position.z =  0.0;

   	chair_surface.meshes.push_back(chair_surface_mesh);
   	chair_surface.mesh_poses.push_back(chair_surface_pose);
   	chair_surface.operation = chair_surface.ADD;

    chair_components.push_back(chair_surface);


    //4 chair legs
    for(int i=0;i<4;i++)
    {
    	moveit_msgs::CollisionObject chair_leg;
    	chair_leg.header.frame_id = "world";
    	chair_leg.id = "chair_leg" + std::to_string(i);
    	shapes::Mesh *leg = shapes::createMeshFromResource("package://srtp_task/mesh/chair_leg.STL",b);
    	shape_msgs::Mesh chair_leg_mesh;
    	shapes::ShapeMsg chair_leg_mesh_msg;
    	shapes::constructMsgFromShape(leg,chair_leg_mesh_msg);
    	chair_leg_mesh = boost::get<shape_msgs::Mesh>(chair_leg_mesh_msg);

    	geometry_msgs::Pose chair_leg_pose;
		chair_leg_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-M_PI/2,0);
	    chair_leg_pose.position.x =  0.6;
	    chair_leg_pose.position.y = 0.30-0.2*i + 0.07;
	    chair_leg_pose.position.z =  -0.1;

	   	chair_leg.meshes.push_back(chair_leg_mesh);
	   	chair_leg.mesh_poses.push_back(chair_leg_pose);
	   	chair_leg.operation = chair_leg.ADD;
	    chair_components.push_back(chair_leg);
    }


    //chair back
    moveit_msgs::CollisionObject chair_back;
	chair_back.header.frame_id = "world";
	chair_back.id = "chair_back";
	shapes::Mesh*back = shapes::createMeshFromResource("package://srtp_task/mesh/chair_back_collision.STL",b);
	shape_msgs::Mesh chair_back_mesh;
	shapes::ShapeMsg chair_back_mesh_msg;
	shapes::constructMsgFromShape(back,chair_back_mesh_msg);
	chair_back_mesh = boost::get<shape_msgs::Mesh>(chair_back_mesh_msg);

	geometry_msgs::Pose chair_back_pose;
	chair_back_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    chair_back_pose.position.x =  0.5;
    chair_back_pose.position.y =  0.5;
    chair_back_pose.position.z =  0.0;

   	chair_back.meshes.push_back(chair_back_mesh);
   	chair_back.mesh_poses.push_back(chair_back_pose);
   	chair_back.operation = chair_back.ADD;
    chair_components.push_back(chair_back);


    planning_scene_interface.addCollisionObjects(chair_components);
}

//装配完成后的椅子
// only for test
void chairSTL(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

	//chair surface
	moveit_msgs::CollisionObject chair_surface;
	chair_surface.header.frame_id = "world";
	chair_surface.id = "chair_surface";
	Eigen::Vector3d b(0.001,0.001,0.001);
	shapes::Mesh*m = shapes::createMeshFromResource("package://srtp_task/mesh/chair_surface.STL",b);
	shape_msgs::Mesh chair_surface_mesh;
	shapes::ShapeMsg chair_surface_mesh_msg;
	shapes::constructMsgFromShape(m,chair_surface_mesh_msg);
	chair_surface_mesh = boost::get<shape_msgs::Mesh>(chair_surface_mesh_msg);

	geometry_msgs::Pose chair_surface_pose;
	chair_surface_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI,0);
    chair_surface_pose.position.x =  0.5;
    chair_surface_pose.position.y =  0.0;
    chair_surface_pose.position.z =  0.0;

   	chair_surface.meshes.push_back(chair_surface_mesh);
   	chair_surface.mesh_poses.push_back(chair_surface_pose);
   	chair_surface.operation = chair_surface.ADD;

    chair_components.push_back(chair_surface);


    //4 chair legs
    for(int i=0;i<4;i++)
    {
    	moveit_msgs::CollisionObject chair_leg;
    	chair_leg.header.frame_id = "world";
    	chair_leg.id = "chair_leg" + std::to_string(i);
    	shapes::Mesh *leg = shapes::createMeshFromResource("package://srtp_task/mesh/chair_leg.STL",b);
    	shape_msgs::Mesh chair_leg_mesh;
    	shapes::ShapeMsg chair_leg_mesh_msg;
    	shapes::constructMsgFromShape(leg,chair_leg_mesh_msg);
    	chair_leg_mesh = boost::get<shape_msgs::Mesh>(chair_leg_mesh_msg);

    	geometry_msgs::Pose chair_leg_pose;
		chair_leg_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI,0);
	    chair_leg_pose.position.x =  0.64 - 0.14*(i/2);
	    chair_leg_pose.position.y =  0.14*(i%2);
	    chair_leg_pose.position.z =  0.0;

	   	chair_leg.meshes.push_back(chair_leg_mesh);
	   	chair_leg.mesh_poses.push_back(chair_leg_pose);
	   	chair_leg.operation = chair_leg.ADD;
	    chair_components.push_back(chair_leg);
    }


    //chair back
    moveit_msgs::CollisionObject chair_back;
	chair_back.header.frame_id = "world";
	chair_back.id = "chair_back";
	shapes::Mesh*back = shapes::createMeshFromResource("package://srtp_task/mesh/chair_back_collision.STL",b);
	shape_msgs::Mesh chair_back_mesh;
	shapes::ShapeMsg chair_back_mesh_msg;
	shapes::constructMsgFromShape(back,chair_back_mesh_msg);
	chair_back_mesh = boost::get<shape_msgs::Mesh>(chair_back_mesh_msg);

	geometry_msgs::Pose chair_back_pose;
	chair_back_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI,0);
    chair_back_pose.position.x =  0.5;
    chair_back_pose.position.y =  0.0;
    chair_back_pose.position.z =  0.0;

   	chair_back.meshes.push_back(chair_back_mesh);
   	chair_back.mesh_poses.push_back(chair_back_pose);
   	chair_back.operation = chair_back.ADD;
    chair_components.push_back(chair_back);


    planning_scene_interface.addCollisionObjects(chair_components);
}



int main(int argc, char **argv)
{
	ros::init(argc,argv,"stl_rviz_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//初始化双臂模型
	dual_arm dual_ur5(node_handle);
	dual_ur5.init();

	//添加椅子模型
	addChairComponents(planning_scene_interface);
	// chairSTL(planning_scene_interface);


    dual_ur5.perform_action(0.65, 0.30, 0.1, 0, M_PI/2, 0, 0.65, -0.10, 0.1, 0, M_PI/2, 0, 0.08, 0.08);
	dual_ur5.grasp(0.04, 0.04);
	dual_ur5.left_gripper.attachObject("chair_leg0");
	dual_ur5.right_gripper.attachObject("chair_leg2");

	// dual_ur5.perform_action(0.33, 0.2, 0.10, 0, 0, -M_PI/2, 0.33, -0.2, 0.10, 0, 0, M_PI/2, 0.04, 0.04);
	dual_ur5.perform_action(0.33, 0.2, 0.07, 0, 0, -M_PI/2, 0.33, -0.2, 0.07, 0, 0, M_PI/2, 0.04, 0.04);
	dual_ur5.grasp(0.08, 0.08);
	dual_ur5.right_gripper.detachObject("chair_leg2");
	dual_ur5.left_gripper.detachObject("chair_leg0");


	dual_ur5.perform_action(0.65, 0.10, 0.1, 0, M_PI/2, 0, 0.65, -0.30, 0.1, 0, M_PI/2, 0, 0.08, 0.08);
	dual_ur5.grasp(0.04, 0.04);
	dual_ur5.left_gripper.attachObject("chair_leg1");
	dual_ur5.right_gripper.attachObject("chair_leg3");

	// dual_ur5.perform_action(0.47, 0.2, 0.10, 0, 0, -M_PI/2, 0.47, -0.2, 0.10, 0, 0, M_PI/2, 0.04, 0.04);
	dual_ur5.perform_action(0.47, 0.2, 0.07, 0, 0, -M_PI/2, 0.47, -0.2, 0.07, 0, 0, M_PI/2, 0.04, 0.04);
	dual_ur5.grasp(0.08, 0.08);
	dual_ur5.right_gripper.detachObject("chair_leg3");
	dual_ur5.left_gripper.detachObject("chair_leg1");


	dual_ur5.perform_action(0.5, 0.57, 0.29, 0, M_PI/2, 0,       0.20, -0.07, 0.07, 0, 0, 0, 0.08, 0.08);
	dual_ur5.grasp(0.045,0.045); 

	//！！！！！！！！！！！！！！玄学警告！！！！！！！！！！！
	//要加sleep，不然有些组件会无法成功attach，我也不知道为什么
	dual_ur5.left_gripper.attachObject("chair_back");
	dual_ur5.right_gripper.attachObject("chair_surface");
	sleep(1);
	dual_ur5.right_gripper.attachObject("chair_leg0");
	sleep(1);
	dual_ur5.right_gripper.attachObject("chair_leg1");
	sleep(1);
	dual_ur5.right_gripper.attachObject("chair_leg2");
	sleep(1);
	dual_ur5.right_gripper.attachObject("chair_leg3");
	dual_ur5.perform_action(0.4, 0.35, 0.21, M_PI/2, 0, -M_PI/2, 0.20, -0.1, 0.07, M_PI/2, 0, 0, 0.08, 0.08);

	double y_left = 0.35, y_right = -0.1;
	while(y_right < y_left)
	{
		if(y_left>0.340) y_left -= 0.005;
		y_right += 0.01;
		bool success = dual_ur5.perform_action(0.4, y_left, 0.21, M_PI/2, 0, -M_PI/2, 0.20, y_right, 0.07, M_PI/2, 0, 0, 0.08, 0.08);
		if(!success) break;
		std::cout << y_left << "  " << y_right << std::endl;
	}

	ros::shutdown();
	return 0;
}