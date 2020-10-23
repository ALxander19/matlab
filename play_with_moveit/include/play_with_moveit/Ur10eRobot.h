#ifndef UR10E_ROBOT
#define UR10E_ROBOT
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit/robot_state/conversions.h>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>


class Ur10eRobot
{
public:
    static const std::string GRIPPER_GROUP;
    static const std::string CAMERA_GROUP;
    static const std::string CHANGER_GROUP;
    static const std::string GRIPPER_EF_GROUP;
    static const std::string GRIPPER_EF_LINK;
    static const std::string CAMERA_EF_LINK;
    static const std::string CHANGER_EF_LINK;
    static const std::vector<std::string> PLANNING_JOINT_NAMES;
    static const std::string FORCE_SENSOR_TOPIC;

    Ur10eRobot(ros::NodeHandle *nh);
    ~Ur10eRobot();

    // function to retrieve force sensor value
    geometry_msgs::WrenchStamped getForceSensorValue();

    // function to move the robot's tool in a straight line with respect to the robot's tool itself.
    // end_point: float array of size 7 [x, y, z, qx, qy, qz, qw]
    // avoid_collisions: whether the planner should avoid collision or not.
    // sometimes when you are performing pick and place, you will want the tool to touch the object.
    // so you will set this arg to false.
    // group_name: specify which planning group you want to move
    void moveToolInStraightLine(double end_point[3], bool avoid_collisions=true, std::string group_name=Ur10eRobot::GRIPPER_GROUP);

    void moveRobotTrajectory(double end_point[3],double ur10e_trajectory_points_x[300],double ur10e_trajectory_points_y[300],double ur10e_trajectory_points_z[300],bool avoid_collisions=true, std::string
    group_name=Ur10eRobot::GRIPPER_GROUP);

    // function to move the robot to pose goal. This function plan, execute the trajectory and wait for the robot to finish it.
    // pose_goal: float array with a size of 3 as [x, y, z],  or 7 as [x, y, z, qx, qy, qz, qw].
    // group_name: string to specify which group to use.
    void goToPoseGoal(std::vector<double> pose_goal, std::string group_name=Ur10eRobot::GRIPPER_GROUP);

    // function to plan and execute a joint-goal-based trajectory. This function also waits for the completion of the trajectory.
    void goToJointGoal(double joint_goal[7]);

    // pick an object using gripper
    // This function assumes that the gripper is already in a place near to the object with an appropriate orientation.
    // So, in order to pick, you will have to approach the object, close the gripper, then retreat.
    void pickObject(std::string object_name, std::string table_name, double approach_dist[3], double retreat_dist[3]);

    // place an object using the gripper.
    // This function assumes that the gripper is already in a place near to the placing location and with an appropriate orientation.
    // so, this function will approach the placing location, open the gripper and then retreat.
    void placeObject(std::string object_name, std::string table_name, double approach_dist[3], double retreat_dist[3]);

    // process output trajectories when needed to remove invalid points from it so it can be executed
    moveit_msgs::RobotTrajectory trajectoryProcessing(moveit_msgs::RobotTrajectory in_traj);

    // open the gripper
    void openGripper();

    // close the gripper
    void closeGripper(double object_width);
};
#endif
