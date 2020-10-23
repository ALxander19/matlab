#include <tuple>
#include <math.h>
#include <iostream>
#include <string>
#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace std;

string input_teleop;


void callback(const std_msgs::String::ConstPtr& msg) {

  input_teleop = msg->data;
}


tuple<double, double, double, double> eulertoquat(double roll, double pitch, double yaw) {

  const int N = 3;
  const int decimals = 10000;

  double** R = new double*[N]; 
  for(int i=0; i<N; i++) {
    R[i] = new double[N];
    for(int j=0; j<N; j++) {
      R[i][j] = 0.0;
    }
  }
  
  R[0][0] = round(cos(yaw)*cos(pitch)*decimals)/decimals;
  R[1][0] = round(sin(yaw)*cos(pitch)*decimals)/decimals;
  R[2][0] = round(-sin(pitch)*decimals)/decimals;
  R[0][1] = round((cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll))*decimals)/decimals;
  R[1][1] = round((sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll))*decimals)/decimals;
  R[2][1] = round(cos(pitch)*sin(roll)*decimals)/decimals;
  R[0][2] = round((cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll))*decimals)/decimals;
  R[1][2] = round((sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll))*decimals)/decimals;
  R[2][2] = round(cos(pitch)*cos(roll)*decimals)/decimals;

  double dEpsilon = 1e-6;
  double a,b,c,d;
  double sign;

  a = 0.5*sqrt(R[0][0]+R[1][1]+R[2][2]+1.0);
  if ( abs(R[0][0]-R[1][1]-R[2][2]+1.0) < dEpsilon ) {
    b = 0.0;
  }
  else {
    if ( R[2][1]-R[1][2] >= 0){ sign = 1; }
    else { sign = -1; }
    b = 0.5*sign*sqrt(R[0][0]-R[1][1]-R[2][2]+1.0);
  }
  if ( abs(R[1][1]-R[2][2]-R[0][0]+1.0) < dEpsilon ) {
    c = 0.0;
  }
  else {
    if ( R[0][2]-R[2][0] >= 0) { sign = 1; }
    else { sign = -1; }
    c = 0.5*sign*sqrt(R[1][1]-R[2][2]-R[0][0]+1.0);
  }
  if ( abs(R[2][2]-R[0][0]-R[1][1]+1.0) < dEpsilon ) {
    d = 0.0;
  }
  else {
    if ( R[1][0]-R[0][1] >= 0) { sign = 1; }
    else { sign = -1; }
    d = 0.5*sign*sqrt(R[2][2]-R[0][0]-R[1][1]+1.0);
  }

  return make_tuple(a,b,c,d);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "test_moveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //moveit::planning_interface::MoveGroupInterface group("manipulator");
  moveit::planning_interface::MoveGroupInterface group("ur10e_electric_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Starting the robot arm teleop");

  ros::Subscriber sub = nh.subscribe("keys", 1000, callback);
  ros::Rate loop_rate(2);

  input_teleop = "0";

  float lin_vel = 0.005, ang_vel = 0.05;

  //float xi = 0.8, yi = 0.0, zi = 0.3;
  float xi = 0.2, yi = -0.2, zi = 1.3;
  float dx = 1*lin_vel, dy = 1*lin_vel, dz = 1*lin_vel;

  //double roll = 0.0, pitch = 1.5708, yaw = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.7854;
  double droll = 1*ang_vel, dpitch = 1*ang_vel, dyaw = 1*ang_vel;
  //double qw, qx, qy, qz;
  //tie(qw,qx,qy,qz) = eulertoquat(roll, pitch, yaw);
  double qw = 0.924 , qx = 0.0, qy = 0.0, qz = 0.383;

  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;
  target_pose.position.x = xi;
  target_pose.position.y = yi;
  target_pose.position.z = zi;

  group.allowReplanning(true);

  while (ros::ok()) {

    tie(qw,qx,qy,qz) = eulertoquat(roll, pitch, yaw);
    //cout << qw << ", " << qx << ", " << qy << ", " << qz << "\n";
    cout << "X: " << target_pose.position.x << ", Y: " << target_pose.position.y << ", Z: " << target_pose.position.z << "\n";
    cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n";

    // Move the robot in X, Y and Z
    if(input_teleop == "w") {

      target_pose.position.z += dz;
    }
     
    if(input_teleop == "s") {

      target_pose.position.z -= dz;
    }

    if(input_teleop == "q") {

      target_pose.position.x += dx;
    }
     
    if(input_teleop == "e") {

      target_pose.position.x -= dx;
    }

    if(input_teleop == "a") {

      target_pose.position.y += dy;
    }
     
    if(input_teleop == "d") {

      target_pose.position.y -= dy;
    }


    // Move the robot in ROLL, PITCH and YAW
    if(input_teleop == "r") {

      roll += droll;
    }
     
    if(input_teleop == "f") {

      roll -= droll;
    }

    if(input_teleop == "t") {

      pitch += dpitch;
    }
     
    if(input_teleop == "g") {

      pitch -= dpitch;
    }

    if(input_teleop == "y") {

      yaw += dyaw;
    }
     
    if(input_teleop == "h") {

      yaw -= dyaw;
    }

    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    group.setNumPlanningAttempts(10);
    group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success) //execute
     throw std::runtime_error("No plan found");

    group.move(); //blocking

    //cout << "From while: " << input_teleop << "\n" ;

    ros::spinOnce();
    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}
