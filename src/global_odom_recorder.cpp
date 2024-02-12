#include <limits>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>

ros::Subscriber odom_sub;
std::ofstream   mapping_file;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("odomCallback");
    mapping_file << msg->pose.pose.position.x << " "
                 << msg->pose.pose.position.y << " "
                 << msg->pose.pose.position.z << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_odom_recorder");
    ros::NodeHandle nh;
    
    mapping_file.open ("/home/patrick/u_turn_ws/data/metalform_carpark.txt");
    odom_sub = nh.subscribe("/odom_global", 1000, odomCallback);
    
    ros::spin();
    return 0;
}


