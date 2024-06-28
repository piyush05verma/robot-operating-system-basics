#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

/// PIYUSH
ros::Publisher pub;
ros::Subscriber sub;

void receiverCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("FROM TIM : %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "piyush");

  ros::NodeHandle nh;

  pub = nh.advertise<std_msgs::String>("sender_topic", 1000);
  sub = nh.subscribe("receiver_topic", 1000, receiverCallback);

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