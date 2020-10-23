#include <ros/ros.h>
#include <matlab_gmm/Num.h>

void callback1(const matlab_gmm::Num& msg1) {

  for (unsigned i=0; i<msg1.arrray.size(); ++i)
    std::cout << "demo1 " << msg1.arrray[i];
    std::cout << '\n';
}

 /// First X then Y ////

void callback2(const matlab_gmm::Num& msg2) {

  for (unsigned i=0; i<msg2.arrray.size(); ++i)
    std::cout << "demo2 " << msg2.arrray[i];
  std::cout << '\n';
}

void callback3(const matlab_gmm::Num& msg3) {

  for (unsigned i=0; i<msg3.arrray.size(); ++i)
    std::cout << "demo3 " << msg3.arrray[i];
  std::cout << '\n';
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subs");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("arrays1", 1000, &callback1);
  ros::Subscriber sub2 = nh.subscribe("arrays2", 1000, &callback2);
  ros::Subscriber sub3 = nh.subscribe("arrays3", 1000, &callback3);
  ros::spin();
}
