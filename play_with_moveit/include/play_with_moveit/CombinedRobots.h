#ifndef COMBINED
#define COMBINED
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

class CombinedRobots
{
public:
    static const std::string COMBINED_GRIPPERS_GROUP;
    static const std::vector<std::string> COMBINED_GRIPPERS_EF_LINKS;
    static const std::vector<std::string> PLANNING_JOINT_NAMES;

    CombinedRobots();
    ~CombinedRobots();

    void goToJointGoal(double joint_goal[13]);
    void goToPoseGoal(std::vector<std::vector<double>>pose_goal);
};
#endif

