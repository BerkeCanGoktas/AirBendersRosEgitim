#include "ros/ros.h"
#include "triangular_angles/Angles.h"

/*
* calculate the third angle and write all the angles to the console
* @param msg -> (const triangular_angles::Angles::ConstPtr&) parameter for getting messages
* @return (void)
*/
void anglesPubCallback(const triangular_angles::Angles::ConstPtr& msg)
{
  ROS_INFO("I heard the first angle as: [%f]", msg->first_angle);
  ROS_INFO("I heard the second angle as: [%f]", msg->second_angle);
  ROS_INFO("So the third angle is: [%f]", (180.0 - (msg->first_angle + msg->second_angle)));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angularListener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/anglesPub", 1000, anglesPubCallback);
  ros::spin();

  return 0;
}
