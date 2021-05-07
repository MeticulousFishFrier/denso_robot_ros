#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_robot_3dp2");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  moveit::planning_interface::MoveGroupInterface move_group("arm");

  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
          std::ostream_iterator<std::string>(std::cout, ", "));
          
  // static const std::string PLANNING_GROUP = "arm";
  // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  // const moveit::core::JointModelGroup* joint_model_group =
  //     move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  

  // robot_model_loader::RobotModelLoaderPtr robot_model_loader(
  //   new robot_model_loader::RobotModelLoader("robot_description"));

  // //begin pilz_industrial_motion_planner using ptp
  // moveit_msgs::MotionPlanRequest request;
  // moveit_msgs::MotionPlanResponse response;
  // geometry_msgs::PoseStamped pose;

  // request.planner_id = std::string("PTP");
  // request.group_name = PLANNING_GROUP;
  // request.max_velocity_scaling_factor = 1;
  // request.max_acceleration_scaling_factor = 1;

  // pose.header.frame_id = "world";
  // pose.pose.position.x = 0.4;
  // pose.pose.position.y = 0.1;
  // pose.pose.position.z = 0.4;
  // pose.pose.orientation.w = 1.0;

  ros::shutdown();

  return 0;
}
