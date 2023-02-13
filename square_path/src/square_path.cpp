#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

//Struct for managing points easily
struct Point
{
    double x,y;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "square_path_node");
    ros::NodeHandle nh;

    //Nodes and Services
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    double altitude = 10;

    //Set square waypoints
    int sampling_degree = 90;
    double r = 10;

    Point p;
    p.x = 0;
    p.y = 0;

    std::vector<Point> points;
    for (int i = 0; i <= 360; i = i + sampling_degree) {
        Point temp;
        temp.x = p.x + r * cos(i * M_PI / 180.0);
        temp.y = p.y + r * sin(i * M_PI / 180.0);

        points.push_back(temp);
    }

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped tkf_pose;
    tkf_pose.pose.position.x = 0;
    tkf_pose.pose.position.y = 0;
    tkf_pose.pose.position.z = altitude;

    //Send some setpoints to connect
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(tkf_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //Take vehicle to the offboard mode, arm it and then wait until 
    //the required altitude is gained
    while(ros::ok() && current_pose.pose.position.z < altitude - 0.5)
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(tkf_pose);

        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;

    //publish the waypoints and wait until they are achieved
    for (int i = 0; i < points.size(); i++) {
        pose.pose.position.x = points[i].x;
        pose.pose.position.y = points[i].y;
        pose.pose.position.z = altitude;

        while (ros::ok() && sqrt(pow(points[i].x - current_pose.pose.position.x, 2) +
                                 pow(points[i].y - current_pose.pose.position.y, 2)) > 1.0) {

            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }

            local_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

    }

    return 0;
}
