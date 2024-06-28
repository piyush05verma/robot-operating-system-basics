#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

/// TIM
ros::Publisher pub;
ros::Subscriber sub;

void senderCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("FROM PIYUSH : %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tim");

  ros::NodeHandle nh;

  pub = nh.advertise<std_msgs::String>("receiver_topic", 1000);
  sub = nh.subscribe("sender_topic", 1000, senderCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg;
    std::string input_msg;

    std::cout << "Type a message to send: ";
    std::getline(std::cin, input_msg);

    msg.data = input_msg;
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}