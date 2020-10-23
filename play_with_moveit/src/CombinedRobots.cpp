#include "play_with_moveit/CombinedRobots.h"

const std::string CombinedRobots::COMBINED_GRIPPERS_GROUP = "combined_grippers";
const std::vector<std::string> CombinedRobots::COMBINED_GRIPPERS_EF_LINKS = {"kuka_vacuum_ef", "ur10e_gripper_ef"};

const std::vector<std::string> CombinedRobots::PLANNING_JOINT_NAMES = {"kuka_joint_a1","kuka_joint_a2","kuka_joint_a3","kuka_joint_a4","kuka_joint_a5",
                    "kuka_joint_a6","kuka_joint_a7","ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
                                "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"};

CombinedRobots::CombinedRobots()
{
}

CombinedRobots::~CombinedRobots()
{
}
void CombinedRobots::goToJointGoal(double joint_goal[13])
{
    moveit::planning_interface::MoveGroupInterface group(CombinedRobots::COMBINED_GRIPPERS_GROUP);
    sensor_msgs::JointState joint_msg;
    joint_msg.name = CombinedRobots::PLANNING_JOINT_NAMES;
    joint_msg.position.assign(joint_goal, joint_goal+13);
    group.setJointValueTarget(joint_msg);
    group.move();
}
void CombinedRobots::goToPoseGoal(std::vector<std::vector<double>> pose_goal)
{
    moveit::planning_interface::MoveGroupInterface group(CombinedRobots::COMBINED_GRIPPERS_GROUP);
    geometry_msgs::Pose pose_msg;

    for (int i; i<2; i++)
    {
        pose_msg.position.x = pose_goal[i][0];
        pose_msg.position.y = pose_goal[i][1];
        pose_msg.position.z = pose_goal[i][2];
        pose_msg.orientation.x = pose_goal[i][3];
        pose_msg.orientation.y = pose_goal[i][4];
        pose_msg.orientation.z = pose_goal[i][5];
        pose_msg.orientation.w = pose_goal[i][6];
        group.setPoseTarget(pose_msg, COMBINED_GRIPPERS_EF_LINKS[i]);
    }
    group.move();
}