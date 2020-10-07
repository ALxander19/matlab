#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <matlab_gmm/Num.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

bool msg_flag = false;
vector<double> x_list; 
vector<double> y_list; 
int i = 0;

void callback(const matlab_gmm::Num& msg) {

  /*for (unsigned i=0; i<msg.arrray.size(); ++i)
    cout << ' ' << msg.arrray[i];
  cout << '\n';*/
  
  //cout << "Points in the trajectory: " << msg.arrray.size() << '\n';

  if (i == 0) {
    
    for (unsigned j = 0; j < msg.arrray.size(); j++)
      x_list.push_back(msg.arrray[j]);
    
  }

  if (i == 1) {
    
    for (unsigned k=0; k<msg.arrray.size(); ++k)
      y_list.push_back(msg.arrray[k]);

    msg_flag = true;
    i = -1;
  }
  
  i++;
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

  ros::init(argc, argv, "test_traj");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("arrays", 1000, &callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  //moveit::planning_interface::MoveGroupInterface group("ur10e_electric_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  while (ros::ok()) {
    
    if (msg_flag) {

      vector<geometry_msgs::Pose> waypoints;

      geometry_msgs::Pose start_pose;
      start_pose.orientation.x = sqrt(2) / 2;
      start_pose.orientation.w = sqrt(2) / 2;
      start_pose.position.x = 0.8;
      start_pose.position.y = 0.0;
      start_pose.position.z = 0.3;

      waypoints.push_back(start_pose);

      geometry_msgs::Pose target_pose = start_pose;

      cout << "Get in!" << endl;
      //cout << "X list: " << x_list.size() << endl;
      //cout << "Y list: " << y_list.size() << endl;

      for (unsigned j = 0; j < x_list.size(); j++) {

        //cout << "Element " << j << " of x list: " << x_list[j] << endl;
        //cout << "Element " << j << " of y list: " << y_list[j] << endl;

        target_pose.position.y -= x_list[j]/10;
        target_pose.position.z -= y_list[j]*10;
        waypoints.push_back(target_pose);
      }

      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      //path execution
      group.execute(trajectory);

      x_list.clear();
      y_list.clear();
      msg_flag = false;
    }
  }

  spinner.stop();
  return 0;
}
