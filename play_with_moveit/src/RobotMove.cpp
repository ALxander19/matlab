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

#include "geometry_msgs/PoseStamped.h"

#include <iomanip>
#include <fstream>
#include <iostream>

#include <nav_msgs/Path.h>

#include <matlab_gmm/Num.h>
#include <ros/ros.h>


std::vector<geometry_msgs::PoseStamped> temp_pose;


void callback1(const matlab_gmm::Num& msg1) {

std::cout<<"Reading Topic1"<<std::endl;

  for (unsigned i=0; i<msg1.arrray.size(); ++i)
    std::cout << "DEMO1 " << msg1.arrray[i];
  std::cout << '\n';
}

void callback2(const matlab_gmm::Num& msg2) {

std::cout<<"Reading Topic2"<<std::endl;

  for (unsigned i=0; i<msg2.arrray.size(); ++i)
    std::cout << "DEMO2 " << msg2.arrray[i];
  std::cout << '\n';
}

void callback3(const matlab_gmm::Num& msg3) {

std::cout<<"Reading Topic3"<<std::endl;
for (unsigned i=0; i<msg3.arrray.size(); ++i)
    std::cout << "DEMO3 " << msg3.arrray[i];
  std::cout << '\n';
}



//void handle_poses(const nav_msgs::Path::ConstPtr& msg)
//{
 // ROS_INFO_STREAM("Received pose: " << msg);
  
  // Use the msg object here to access the pose elements,
  // like msg->pose.pose.position.x

  //temp_pose = msg->poses;

 // float size = msg->poses.size();
  
  //std::cout<< "X: "<<msg->poses[0].pose.position.x<< std::endl;
  //std::cout<< "Y:" <<msg->poses[0].pose.position.y<< std::endl;
 // std::cout<< "Z:" <<msg->poses[0].pose.position.z<< std::endl;
  
  //temp_pose.push_back(msg);
//}


int main(int argc, char** argv)
{
  
      //Reading FILES
     
  double x ;
 double y ;
  double z ;
 double xx ;
  double yy ;
  double zz ;
  double aa ;
  double l,m,nn; 

   double a[300], b[300], c[300];//size of array more than number of entries in data file

    std::ifstream infile;
    infile.open("/home/sachink/catkin_ws/src/play_with_moveit/src/trajec.txt");//open the text file
    
       if (!infile) 
    {
        std::cout << "Unable to open file";
       exit(1); // terminate with error
    }
      int i=1;
    while ( i<=124)//!infile.eof())
    {
      std::cout<<"Counter::"<<i<<std::endl;
    //To make three arrays for each column (a for 1st column, b for 2nd....)
    infile>>aa>>xx>>yy>>zz>>l>>m>>nn;
   
    a[i]=xx;
    b[i]=yy;
    c[i]=zz;
     
    //std::cout<<a[i]<<"\t"<< b[i]<<"\t"<<c[i]<<std::endl;
    i= i+1;
   }         
          //std::cout<<a[1]<<"\t"<< b[1]<<"\t"<<c[1]<<std::endl;
          //std::cout<<a[2]<<"\t"<< b[2]<<"\t"<<c[2]<<std::endl;
          //std::cout<<a[3]<<"\t"<< b[3]<<"\t"<<c[3]<<std::endl;
          //std::cout<<a[4]<<"\t"<< b[4]<<"\t"<<c[4]<<std::endl;
          //std::cout<<a[5]<<"\t"<< b[5]<<"\t"<<c[5]<<std::endl;
          
          infile.close(); 
     // //
          
      double ur10e_initial_location[3] = {a[1],b[1],c[1]};
      std::cout<<"YY"<<a[1]<<"\t"<<b[1]<<"\t"<<c[1]<<std::endl;

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
    //std::cout<< "POSE_UR10e_X: "<<ur10e_gripper_pose[0]<< std::endl; [0.4] 
    //std::cout<< "POSE_UR10e_Y: "<<ur10e_gripper_pose[1]<< std::endl; [-0.45]
    //std::cout<< "POSE_UR10e_Z: "<<ur10e_gripper_pose[2]<< std::endl; [1.15 ]
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

    //std::cout<< "POSE_KUKA_X: "<<kuka_vacuum_pose[0]<< std::endl;  [0]
    //std::cout<< "POSE_KUKA_Y: "<<kuka_vacuum_pose[1]<< std::endl;  [0.4]
    //std::cout<< "POSE_KUKA_Z: "<<kuka_vacuum_pose[2]<< std::endl;  [1.27]  
    // go to the place location
    double kuka_place_location[3] = {0.2, 0.0, 0.0};

    // place the object
    double kuka_place_approach_dist[3] = {0.0, 0, -0.1};
    double kuka_place_retreat_dist[3] = {0.0, 0, 0.2};
      
  //  std::cout<< "BEFORE:";
  //  ros::Subscriber sub = nh.subscribe("/trajectory",1000, handle_poses);
  //  std::cout<< "AFTER:";

  
    // start motion
    //
    std::vector<std::vector<double>> combined_pick_pose;
    //combined_pick_pose.push_back(kuka_vacuum_pose);
    combined_pick_pose.push_back(ur10e_gripper_pose);
    //combined_robots.goToPoseGoal(combined_pick_pose);

    // pick the object
   
    // ur10e.pickObject(ur10e_object_name, ur10e_table_name, ur10e_pick_approach_dist, ur10e_pick_retreat_dist);
    // kuka.pickObject(kuka_object_name, kuka_table_name, kuka_pick_approach_dist, kuka_pick_retreat_dist);

     std::cout<<"XX"<<a[1]<<"\t"<<b[1]<<"\t"<<c[1]<<std::endl;
     ur10e.moveRobotTrajectory(ur10e_place_location,a,b,c);
     //ur10e.moveToolInStraightLine(ur10e_place_location);
     //kuka.moveToolInStraightLine(kuka_place_location);
         //  std::cout<< "PPPPP:::";

     //ur10e.placeObject(ur10e_object_name, ur10e_table_name, ur10e_place_approach_dist, ur10e_place_retreat_dist);
            // std::cout<< "QQQQQ::";    
     //kuka.placeObject(kuka_object_name, kuka_table_name, kuka_place_approach_dist, kuka_place_retreat_dist);

    std::cout<< "BEFORE CALL:::";
    //ros::Subscriber sub1 = nh.subscribe("arrays1", 1000, &callback1);
    //ros::Subscriber sub2 = nh.subscribe("arrays2", 1000, &callback2);   
    //ros::Subscriber sub3 = nh.subscribe("arrays3", 1000, &callback3);
    std::cout<< "AFTER CALL:::";

    return 0;
}

