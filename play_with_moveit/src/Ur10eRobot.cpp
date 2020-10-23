#include"play_with_moveit/Ur10eRobot.h"

const std::string Ur10eRobot::GRIPPER_GROUP = "ur10e_electric_gripper";
const std::string Ur10eRobot::GRIPPER_EF_GROUP = "ur10e_gripper_ef";
const std::string Ur10eRobot::CHANGER_GROUP = "ur10e_camera";
const std::string Ur10eRobot::CAMERA_GROUP = "ur10e_tool_changer";
const std::string Ur10eRobot::GRIPPER_EF_LINK = "ur10e_gripper_ef";
const std::string Ur10eRobot::CAMERA_EF_LINK = "ur10e_camera_ef";
const std::string Ur10eRobot::CHANGER_EF_LINK = "ur10e_changer_ef";
const std::vector<std::string> Ur10eRobot::PLANNING_JOINT_NAMES{"ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
                                                                "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"};
const std::string Ur10eRobot::FORCE_SENSOR_TOPIC = "/ur10e_force_topic";

Ur10eRobot::Ur10eRobot(ros::NodeHandle *nh)
{
}

Ur10eRobot::~Ur10eRobot()
{
}

geometry_msgs::WrenchStamped Ur10eRobot::getForceSensorValue()
{
    geometry_msgs::WrenchStamped::ConstPtr current_force_value;
    current_force_value = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(Ur10eRobot::FORCE_SENSOR_TOPIC);
    return *current_force_value;
}
void Ur10eRobot::moveToolInStraightLine(double end_point[3],bool avoid_collisions, std::string group_name)
{

    moveit::planning_interface::MoveGroupInterface group(group_name);
    geometry_msgs::Pose way_point;
    std::vector<geometry_msgs::Pose> way_points_list;

    // push back the starting point always zeros with respect to the tool.
    way_point.position.x = 0;
    way_point.position.y = 0;
    way_point.position.z = 0;
    way_point.orientation.x = 0;
    way_point.orientation.y = 0;
    way_point.orientation.z = 0;
    way_point.orientation.w = 0;
    way_points_list.push_back(way_point);

    // push back the end point
    way_point.position.x = end_point[0];
    way_point.position.y = end_point[1];
    way_point.position.z = end_point[2];
    way_points_list.push_back(way_point);

    // use the required group to plan the cartesian path
    moveit_msgs::RobotTrajectory out_traj;
    double cartesian_success;

    // set the reference frame to the tool
    group.setPoseReferenceFrame(Ur10eRobot::GRIPPER_EF_LINK);

    // plan the cartesian path
    cartesian_success = group.computeCartesianPath(way_points_list, 0.01, 0.0, out_traj, false);

    // process the trajectory to remove invalid points
    out_traj = trajectoryProcessing(out_traj);
    
    // get the current robot state
    moveit_msgs::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*group.getCurrentState(), robot_state_msg);

    // execute the planned path
    const moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan{robot_state_msg, out_traj, 10};
    group.execute(cartesian_plan);
}

void Ur10eRobot::moveRobotTrajectory(double end_point[3],double ur10e_trajectory_points_x[300],double ur10e_trajectory_points_y[300],double ur10e_trajectory_points_z[300],bool avoid_collisions, std::string group_name)
{
    std::cout<<"INSIDE !!!"<<std::endl;
    //std::cout<<end_point[1]<<"\t"<< end_point[2]<<"\t"<<end_point[3]<<std::endl;
    moveit::planning_interface::MoveGroupInterface group(group_name);
    geometry_msgs::Pose way_point;
    std::vector<geometry_msgs::Pose> way_points_list;

    // push back the starting point always zeros with respect to the tool.
    way_point.position.x = 0;
    way_point.position.y = 0;
    way_point.position.z = 0;
    way_point.orientation.x = 0;
    way_point.orientation.y = 0;
    way_point.orientation.z = 0;
    way_point.orientation.w = 0;
    way_points_list.push_back(way_point);

    // push back the end point
    for(int i=1; i<=300;i++)
    {
     way_point.position.x = ur10e_trajectory_points_x[i];
     way_point.position.y = ur10e_trajectory_points_y[i];
     way_point.position.z = ur10e_trajectory_points_z[i];
     way_points_list.push_back(way_point);
    }

    // use the required group to plan the cartesian path
    moveit_msgs::RobotTrajectory out_traj;
    double cartesian_success;

    // set the reference frame to the tool
    group.setPoseReferenceFrame(Ur10eRobot::GRIPPER_EF_LINK);

    // plan the cartesian path
    cartesian_success = group.computeCartesianPath(way_points_list, 0.01, 0.0, out_traj, false);

    // process the trajectory to remove invalid points
    out_traj = trajectoryProcessing(out_traj);
    
    // get the current robot state
    moveit_msgs::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*group.getCurrentState(), robot_state_msg);

    // execute the planned path
    const moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan{robot_state_msg, out_traj, 10};
    group.execute(cartesian_plan);
}

void Ur10eRobot::goToJointGoal(double joint_goal[7])
{
    moveit::planning_interface::MoveGroupInterface group(Ur10eRobot::GRIPPER_GROUP);
    sensor_msgs::JointState joint_msg;
    joint_msg.name = Ur10eRobot::PLANNING_JOINT_NAMES;
    joint_msg.position.assign(joint_goal, joint_goal+7);
    group.setJointValueTarget(joint_msg);
    group.move();
}

void Ur10eRobot::goToPoseGoal(std::vector<double> pose_goal, std::string group_name)
{
    moveit::planning_interface::MoveGroupInterface group(group_name);
    geometry_msgs::Pose pose_msg;

    pose_msg.position.x = pose_goal[0];
    pose_msg.position.y = pose_goal[1];
    pose_msg.position.z = pose_goal[2];

    if (pose_goal.size() == 7)
    {
        pose_msg.orientation.x = pose_goal[3];
        pose_msg.orientation.y = pose_goal[4];
        pose_msg.orientation.z = pose_goal[5];
        pose_msg.orientation.w = pose_goal[6];
    }
    else
    {
        ROS_INFO("invalid input for pose goal");
    }

    group.setPoseTarget(pose_msg);
    group.move();
}
moveit_msgs::RobotTrajectory Ur10eRobot::trajectoryProcessing(moveit_msgs::RobotTrajectory in_traj)
{
    for(int i; i < in_traj.joint_trajectory.points.size()-1; i++)
    {
        if(in_traj.joint_trajectory.points[i+1].time_from_start.toNSec() <= in_traj.joint_trajectory.points[i].time_from_start.toNSec())
        {
            // delete i+1 point
            in_traj.joint_trajectory.points.erase(in_traj.joint_trajectory.points.begin()+i+1);
        }
    }    
    return in_traj;
}
void Ur10eRobot::pickObject(std::string object_name, std::string table_name, double approach_dist[3], double retreat_dist[3])
{
    // initialize  gripper group
    moveit::planning_interface::MoveGroupInterface gripper_group(Ur10eRobot::GRIPPER_GROUP);

    // disable collision betweeen the table and the ef.
    gripper_group.setSupportSurfaceName(table_name);

    // approach the object
    moveToolInStraightLine(approach_dist);

    // attach the object
    gripper_group.attachObject(object_name);

    // close the gripper
    ros::Duration(2).sleep();

    // retreat
    moveToolInStraightLine(retreat_dist);
}
void Ur10eRobot::placeObject(std::string object_name, std::string table_name, double approach_dist[3], double retreat_dist[3])
{
    // initialize gripper group
    moveit::planning_interface::MoveGroupInterface gripper_group(Ur10eRobot::GRIPPER_GROUP);

    // disable collision betweeen the table and the ef.
    gripper_group.setSupportSurfaceName(table_name);

    // approach the place location
    moveToolInStraightLine(approach_dist);

    // detach the object
    gripper_group.detachObject(object_name);

    // open the gripper
    ros::Duration(2).sleep();

    // retreat
    moveToolInStraightLine(retreat_dist);
}
void Ur10eRobot::openGripper()
{
    moveit::planning_interface::MoveGroupInterface group(Ur10eRobot::GRIPPER_EF_GROUP);
    std::vector<double> joint_goal{0.0, 0.0};
    group.setJointValueTarget(joint_goal);
    group.move();
}
void Ur10eRobot::closeGripper(double object_width)
{
    double MAX_WIDTH = 0.02;
    if (object_width > MAX_WIDTH)
        ROS_INFO("cannot close gripper, object width exceeds the max width");
    else
    {
        double gripper_dist = (MAX_WIDTH - object_width) / 2.0;
        moveit::planning_interface::MoveGroupInterface group(Ur10eRobot::GRIPPER_EF_GROUP);
        std::vector<double> joint_goal{-gripper_dist, -gripper_dist};
        group.setJointValueTarget(joint_goal);
        group.move();
    }
}
