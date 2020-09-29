#include <ros/ros.h>
#include <matlab_gmm/Num.h>

void callback(const matlab_gmm::Num& msg) {

  for (unsigned i=0; i<msg.arrray.size(); ++i)
    std::cout << ' ' << msg.arrray[i];
  std::cout << '\n';
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subs");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("arrays", 1000, &callback);
  ros::spin();
}
