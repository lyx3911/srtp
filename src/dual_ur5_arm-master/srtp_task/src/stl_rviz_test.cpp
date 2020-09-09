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
		chair_leg_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-M_PI/2,0);
	    chair_leg_pose.position.x =  0.7;
	    chair_leg_pose.position.y =  -0.15+0.1*i;
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

int main(int argc, char **argv)
{
	ros::init(argc,argv,"stl_rviz_test");
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

	addChairComponents(planning_scene_interface);
	ros::shutdown();
	return 0;
}