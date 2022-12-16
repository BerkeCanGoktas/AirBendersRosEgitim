#include <iostream>

#include "ros/ros.h"
#include "triangular_angles/Angles.h"

float getInput();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angularTalker");
  ros::NodeHandle n;
  ros::Publisher angles_pub = n.advertise<triangular_angles::Angles>("/anglesPub", 1000);

  if(ros::ok())
  {
    triangular_angles::Angles msg;

    std::cout << "Please enter the first angle: ";
    msg.first_angle = getInput();
    std::cout << "Please enter the second angle: ";
    msg.second_angle = getInput();
    ROS_INFO("First angle: [%f]", msg.first_angle);
    ROS_INFO("Second angle: [%f]", msg.second_angle);

    angles_pub.publish(msg);
    ros::spinOnce();
  }


  return 0;
}


/*
* take input from the user
* @return (float) user input in correct format
*/
float getInput()
{
    while (true)
    {
        float t{};
        std::cin >> t;
        if (!std::cin)
        {
            std::cin.clear();
            std::cerr << "Please enter a valid angle and try again!" << '\n';
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        else
        {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return t;
        }
    }
}
