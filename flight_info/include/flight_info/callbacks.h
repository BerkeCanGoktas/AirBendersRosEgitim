#ifndef CB_H
#define CB_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace cb
{
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::TwistStamped current_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel = *msg;
}

sensor_msgs::Imu current_imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_imu = *msg;
}

mavros_msgs::AttitudeTarget current_att;
void att_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    current_att = *msg;
}

geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

sensor_msgs::NavSatFix current_glob;
void glob_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_glob = *msg;
}
}

#endif