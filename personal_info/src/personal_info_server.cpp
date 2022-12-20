#include <string>

#include "ros/ros.h"
#include "personal_info/PersonalInfo.h"

bool message(personal_info::PersonalInfo::Request  &req,
             personal_info::PersonalInfo::Response &res)
{
    res.message = "My name is " + req.name + ". I am " + std::to_string(req.age) + " years old.";
    ROS_INFO("request: name=%s, age=%d", req.name.c_str(), (int)req.age);
    ROS_INFO("sending back response: [%s]", res.message.c_str());
    return true;
}

void shutdown(const ros::TimerEvent&)
{
  ROS_INFO("Time is up! Server is shutting down.");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "personal_info_server");
  ros::NodeHandle n;
  ros::Timer shutdown_timer = n.createTimer(ros::Duration(30), shutdown);
  ros::ServiceServer service = n.advertiseService("personal_info_server", message);
  ROS_INFO("Ready to send message.");
  ros::spin();

  return 0;
}

