#include "ros/ros.h"
#include "personal_info/PersonalInfo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "personal_info_client");
  if (argc != 3)
  {
    ROS_INFO("usage: personal_info_client name age");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<personal_info::PersonalInfo>("personal_info_server");
  personal_info::PersonalInfo srv;
  srv.request.name = argv[1];
  srv.request.age = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Message: %s", srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service personal_info");
    return 1;
  }

  return 0;
}

