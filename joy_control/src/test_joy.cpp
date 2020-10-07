#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

using namespace std;

std_msgs::String msg;


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

  msg.data = "0";

  if(joy->buttons[4] == 1) {   // Deadman button
    
    if(joy->buttons[0] == 1) { // X is press
      
      if(joy->axes[1] >= 0.1) { // Up axis
        //cout << "X UP\n";
        msg.data = "q";
      }
      
      if(joy->axes[1] <= -0.1) { // Down axis
        //cout << "X DOWN\n";
        msg.data = "e";
      }

      if(joy->axes[1] > -0.1 && joy->axes[1] < 0.1) { // Nothing axis
        //cout << "X NONE\n";
        msg.data = "0";
      }

    }

    if(joy->buttons[3] == 1) { // Z is press
      
      if(joy->axes[1] >= 0.1) { // Up axis
        //cout << "Z UP\n";
        msg.data = "w";
      }
      
      if(joy->axes[1] <= -0.1) { // Down axis
        //cout << "Z DOWN\n";
        msg.data = "s";
      }

      if(joy->axes[1] > -0.1 && joy->axes[1] < 0.1) { // Nothing axis
        //cout << "Z NONE\n";
        msg.data = "0";
      }

    }

    if(joy->buttons[2] == 1) { // Y is press
      
      if(joy->axes[1] >= 0.1) { // Up axis
        //cout << "Y UP\n";
        msg.data = "a";
      }
      
      if(joy->axes[1] <= -0.1) { // Down axis
        //cout << "Y DOWN\n";
        msg.data = "d";
      }

      if(joy->axes[1] > -0.1 && joy->axes[1] < 0.1) { // Nothing axis
        //cout << "Y NONE\n";
        msg.data = "0";
      }

    }

    if(joy->buttons[13] == 1) { // UP is press
      
      if(joy->axes[4] >= 0.1) { // Up axis
        msg.data = "r";
      }
      
      if(joy->axes[4] <= -0.1) { // Down axis
        msg.data = "f";
      }

      if(joy->axes[4] > -0.1 && joy->axes[4] < 0.1) { // Nothing axis
        msg.data = "0";
      }

    }

    if(joy->buttons[16] == 1) { // RIGHT is press
      
      if(joy->axes[4] >= 0.1) { // Up axis
        msg.data = "t";
      }
      
      if(joy->axes[4] <= -0.1) { // Down axis
        msg.data = "g";
      }

      if(joy->axes[4] > -0.1 && joy->axes[4] < 0.1) { // Nothing axis
        msg.data = "0";
      }

    }

    if(joy->buttons[14] == 1) { // DOWN is press
      
      if(joy->axes[4] >= 0.1) { // Up axis
        msg.data = "y";
      }
      
      if(joy->axes[4] <= -0.1) { // Down axis
        msg.data = "h";
      }

      if(joy->axes[4] > -0.1 && joy->axes[4] < 0.1) { // Nothing axis
        msg.data = "0";
      }

    }

  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joy");
  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
  ros::Publisher key_pub = nh.advertise<std_msgs::String>("keys", 1000);

  ros::Rate loop_rate(5);

  msg.data = "0";

  while (ros::ok()) {

    key_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();
    loop_rate.sleep();

  }

  //ros::spin();
  return 0;
}
