#include "ros/ros.h"
#include "game_master/send_command.h"
#include "std_msgs/String.h"


bool add(game_master::send_command::Request  &req,
         game_master::send_command::Response &res)
{
  res.answ = "hab dich gehört";//req.start;
  ROS_INFO("I got : %s", req.start.c_str());
  ROS_INFO("sending back response: [%s]", res.answ.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_command");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("send_command", add);
  ROS_INFO("Ready to receive command.");
  ros::spin();

  return 0;
}