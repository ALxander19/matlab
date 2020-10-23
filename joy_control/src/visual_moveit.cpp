#include <string>
#include <iostream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace std;


int main(int argc, char** argv) {

  ros::init(argc, argv, "test_moveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("manipulator");

  ROS_INFO("Starting the robot arm teleop");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.05;

  double xi = 0.8, yi = 0.0, zi = 0.3;
  double qw = 0.924 , qx = 0.0, qy = 0.0, qz = 0.383;

  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;
  target_pose.position.x = xi;
  target_pose.position.y = yi;
  target_pose.position.z = zi;

  group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(!success) //execute
    throw std::runtime_error("No plan found");

  group.move(); //blocking

  ros::Rate loop_rate(1);

  while (ros::ok()) {

    cout << "Get in!" << endl;

    visual_tools.publishAxisLabeled(target_pose, "pose");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    ros::spinOnce();
    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}
