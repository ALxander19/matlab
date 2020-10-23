#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include "play_with_moveit/KukaRobot.h"
#include "play_with_moveit/Ur10eRobot.h"
#include "play_with_moveit/CombinedRobots.h"
#include <cmath>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv)
{
    // //
    // // initialization
    // //
    ros::init(argc, argv, "moveit_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(3).sleep();
    
    CombinedRobots combined_robots;
    Ur10eRobot ur10e(&nh);
    KukaRobot kuka(&nh);
    // variables to get info about the environment
    std_msgs::Float32MultiArray::ConstPtr get_array_msg;
    std_msgs::String::ConstPtr get_string_msg;

    //
    // decide pick and place parameters for ur10e
    //
    // information about the table
    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("ur10e_table_size");
    double ur10e_table_size[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("ur10e_table_pose");
    double ur10e_table_pose_list[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_string_msg = ros::topic::waitForMessage<std_msgs::String>("ur10e_table_name");
    std::string ur10e_table_name = get_string_msg->data;

    // information about the object
    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("ur10e_object_size");
    double ur10e_object_size[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("ur10e_object_pose");
    double ur10e_object_pose_list[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};
    
    get_string_msg = ros::topic::waitForMessage<std_msgs::String>("ur10e_object_name");
    std::string ur10e_object_name = get_string_msg->data;

    // go to the pick location
    double ur10e_pick_approach_dist[3] = {0, 0.03, 0};
    double ur10e_pick_retreat_dist[3] = {0, -0.1, 0.1};
    double ur10e_contact_margin = 0.05;
    std::vector<double>  ur10e_gripper_pose{ur10e_object_pose_list[0], ur10e_object_pose_list[1] - ur10e_contact_margin, ur10e_object_pose_list[2], 0, 0, 0, 1};
    // go to the place location
    double ur10e_place_location[3] = {0.2, 0.0, 0.0};

    // place the object
    double ur10e_place_approach_dist[3] = {0, 0.1, -0.1};
    double ur10e_place_retreat_dist[3] = {0, -0.1, 0.1};

       
    //
    // decide pick and place parameters for kuka
    //
    // information about the table
    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("kuka_object_size");
    double kuka_table_size[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("kuka_object_size");
    double kuka_table_pose_list[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_string_msg = ros::topic::waitForMessage<std_msgs::String>("kuka_object_name");
    std::string kuka_table_name = get_string_msg->data;

    // information about the object
    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("kuka_object_size");
    double kuka_object_size[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};

    get_array_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("kuka_object_pose");
    double kuka_object_pose_list[3] = {get_array_msg->data[0], get_array_msg->data[1], get_array_msg->data[2]};
    
    get_string_msg = ros::topic::waitForMessage<std_msgs::String>("kuka_object_name");
    std::string kuka_object_name = get_string_msg->data;

    // go to the picking pose
    double kuka_pick_approach_dist[3] = {0.0, 0.0, -0.05};
    double kuka_pick_retreat_dist[3] = {0.0, 0.0, 0.1};
    double kuka_contact_margin = 0.02;
    
    double kuka_vacuum_height = kuka_object_pose_list[2] + kuka_object_size[2]/2.0 + kuka_contact_margin - kuka_pick_approach_dist[2];
    std::vector<double> kuka_vacuum_pose{kuka_object_pose_list[0], kuka_object_pose_list[1], kuka_vacuum_height, 0, 0, 0, 1};
    // go to the place location
    double kuka_place_location[3] = {0.2, 0.0, 0.0};

    // place the object
    double kuka_place_approach_dist[3] = {0.0, 0, -0.1};
    double kuka_place_retreat_dist[3] = {0.0, 0, 0.2};

    //
    // start motion
    //
    std::vector<std::vector<double>> combined_pick_pose;
    combined_pick_pose.push_back(kuka_vacuum_pose);
    combined_pick_pose.push_back(ur10e_gripper_pose);
    combined_robots.goToPoseGoal(combined_pick_pose);

    // pick the object
    ur10e.pickObject(ur10e_object_name, ur10e_table_name, ur10e_pick_approach_dist, ur10e_pick_retreat_dist);
    kuka.pickObject(kuka_object_name, kuka_table_name, kuka_pick_approach_dist, kuka_pick_retreat_dist);

    ur10e.moveToolInStraightLine(ur10e_place_location);
    kuka.moveToolInStraightLine(kuka_place_location);

    ur10e.placeObject(ur10e_object_name, ur10e_table_name, ur10e_place_approach_dist, ur10e_place_retreat_dist);
    kuka.placeObject(kuka_object_name, kuka_table_name, kuka_place_approach_dist, kuka_place_retreat_dist);

    return 0;
}

