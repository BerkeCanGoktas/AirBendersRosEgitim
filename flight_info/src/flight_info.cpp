#include <array>        //for std::array
#include <fstream>      //for std::ifstream
#include <string>       //for std::string
#include <vector>       //for std::vector

#include <ros/ros.h>

#include <common_pkg/WGS84toCartesian.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "flight_info/callbacks.h"

void calculateRPY();
void getCoordinatesFromFile(std::string filePath);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flight_info_node");
    ros::NodeHandle nh;

    //Ros subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, cb::state_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("mavros/local_position/velocity", 10, cb::vel_cb);
    ros::Subscriber acc_sub = nh.subscribe<sensor_msgs::Imu>
        ("mavros/imu/data", 10, cb::imu_cb);
        ros::Subscriber thrust_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
        ("mavros/setpoint_raw/target_attitude", 10, cb::att_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, cb::pos_cb);
    ros::Subscriber glob_sub = nh.subscribe<sensor_msgs::NavSatFix>
        ("mavros/global_position/global", 10, cb::glob_cb);

    ros::Rate rate(1.0);
    bool once = true;

    while(ros::ok())
    {
        //try to connect
        while(ros::ok() && !cb::current_state.connected)
        {
            ROS_INFO("Trying to connect...");
            ros::spinOnce();
            rate.sleep();
        }

        //after connection print transformed file coordinates for once
        while(once)
        {
            once = false;
            ROS_INFO("Connected!\n");
            getCoordinatesFromFile("/home/berke/catkin_ws/src/flight_info/coordinates.txt");
        }

        //print flying data
        ROS_INFO("Local Position:");
        ROS_INFO("x: %f y: %f z: %f\n", cb::current_pos.pose.position.x,
            cb::current_pos.pose.position.y, cb::current_pos.pose.position.z);

        ROS_INFO("Global Position:");
        ROS_INFO("longitude: %f latitude: %f altitude: %f\n", cb::current_glob.longitude,
            cb::current_glob.latitude, cb::current_glob.altitude);

        ROS_INFO("Velocity:");
        ROS_INFO("x: %f y: %f z: %f\n", cb::current_vel.twist.linear.x,
            cb::current_vel.twist.linear.y, cb::current_vel.twist.linear.z);

        ROS_INFO("Acceleration:");
        ROS_INFO("x: %f y: %f z: %f\n", cb::current_imu.linear_acceleration.x,
            cb::current_imu.linear_acceleration.y, cb::current_imu.linear_acceleration.z);
        
        ROS_INFO("Thrust: %f\n", cb::current_att.thrust);

        calculateRPY();
        ros::spinOnce();
        rate.sleep();
    }
    
}

/*
* Reads latitude and longitude from given file path and transforms it
* to cartesian using the mavros global location data, then prints it to
* the terminal
* @param filePath -> (std::string) path of file for reading latitude and longitude
*/
void getCoordinatesFromFile(std::string filePath)
{
    std::string txt;
    std::vector<std::array<double, 2>> coordinates;
    std::ifstream file(filePath);
    while (getline(file, txt)) 
    {
        std::string str2 (",");
        std::string substr;
        std::size_t found = txt.find(str2);
        if (found!=std::string::npos)
        {
            substr = txt.substr(1, found - 1);
            double lon = stod(substr);
            substr = txt.substr(found + 1, txt.length() - 1);
            substr.erase(substr.end() - 1, substr.end());
            double lat = stod(substr);
            coordinates.push_back({lon, lat});
        }
    }
    for(std::array<double,2> coordinate : coordinates)
    {
        std::array<double, 2> wgs84Coord = wgs84::toCartesian(
            {cb::current_glob.latitude, cb::current_glob.longitude}, coordinate);
        ROS_INFO("Transformed x and y: %f, %f\n", wgs84Coord.at(0), wgs84Coord.at(1));
    }
    file.close();
}

/*
* Calculates roll, pitch, yaw from orientation and prints these data to the terminal
*/
void calculateRPY()
{
    geometry_msgs::Quaternion q = cb::current_pos.pose.orientation;
    float roll  = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    float pitch = asin(2.0 * (q.z * q.x - q.w * q.y));
    float yaw   = atan2(2.0 * (q.w * q.x + q.y * q.z) , - 1.0 + 2.0 * (q.x * q.x + q.y * q.y));

    ROS_INFO("roll: %f pitch: %f yaw: %f \n", roll, pitch, yaw);
}
