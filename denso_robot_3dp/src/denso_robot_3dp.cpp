#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

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

// ros::init(argc, argv, "move_group_tutorial");
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   ros::NodeHandle node_handle("~");

//   // BEGIN_TUTORIAL
//   // Start
//   // ^^^^^
//   // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
//   // a RobotModel and a PlanningScene.
//   //
//   // We will start by instantiating a `RobotModelLoader`_ object, which will look up the robot description on the ROS
//   // parameter server and construct a :moveit_core:`RobotModel` for us to use.
//   //
//   // .. _RobotModelLoader:
//   //     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html

//   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//   robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  

//   // Using the :moveit_core:`RobotModel`, we can construct a
//   // :planning_scene:`PlanningScene` that maintains the state of
//   // the world (including the robot).
//   planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

//   // We can now setup the PlanningPipeline
//   // object, which will use the ROS parameter server
//   // to determine the set of request adapters and the
//   // planning plugin to use
//   planning_pipeline::PlanningPipelinePtr planning_pipeline(
//       new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

//   // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools("link_1");
//   visual_tools.deleteAllMarkers();

//   /* Remote control is an introspection tool that allows users to step through a high level script
//      via buttons and keyboard shortcuts in RViz */
//   visual_tools.loadRemoteControl();

//   /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

//   /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
//   visual_tools.trigger();

//   /* Sleep a little to allow time to startup rviz, etc..
//      This ensures that visual_tools.prompt() isn't lost in a sea of logs*/
//   ros::Duration(10).sleep();

//   /* We can also use visual_tools to wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Pose Goal
//   // ^^^^^^^^^
//   // We will now create a motion plan request for the right arm of the Panda
//   // specifying the desired pose of the end-effector as input.
//   planning_interface::MotionPlanRequest req;
//   planning_interface::MotionPlanResponse res;
//   geometry_msgs::PoseStamped pose;
//   // pose.header.frame_id = "world";
//   // pose.pose.position.x = 0.3;
//   // pose.pose.position.y = 0.0;
//   // pose.pose.position.z = 0.75;
//   // pose.pose.orientation.w = 1.0;

//   pose.header.frame_id = "world";
//   pose.pose.position.x = 0.4;
//   pose.pose.position.y = 0.1;
//   pose.pose.position.z = 0.4;
//   pose.pose.orientation.w = 1.0;

//   // A tolerance of 0.01 m is specified in position
//   // and 0.01 radians in orientation
//   std::vector<double> tolerance_pose(3, 0.01);
//   std::vector<double> tolerance_angle(3, 0.01);

//   // We will create the request as a constraint using a helper function available
//   // from the
//   // `kinematic_constraints`_
//   // package.
//   //
//   // .. _kinematic_constraints:
//   //     http://docs.ros.org/melodic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
//   req.group_name = "panda_arm";
//   moveit_msgs::Constraints pose_goal =
//       kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
//   req.goal_constraints.push_back(pose_goal);

//   // Now, call the pipeline and check whether planning was successful.
//   planning_pipeline->generatePlan(planning_scene, req, res);
//   /* Check that the planning was successful */
//   if (res.error_code_.val != res.error_code_.SUCCESS)
//   {
//     ROS_ERROR("Could not compute plan successfully");
//     return 0;
//   }

//   // Visualize the result
//   // ^^^^^^^^^^^^^^^^^^^^
//   ros::Publisher display_publisher =
//       node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//   moveit_msgs::DisplayTrajectory display_trajectory;

//   /* Visualize the trajectory */
//   ROS_INFO("Visualizing the trajectory");
//   moveit_msgs::MotionPlanResponse response;
//   res.getMessage(response);

//   display_trajectory.trajectory_start = response.trajectory_start;
//   display_trajectory.trajectory.push_back(response.trajectory);
//   display_publisher.publish(display_trajectory);

  return 0;
}
