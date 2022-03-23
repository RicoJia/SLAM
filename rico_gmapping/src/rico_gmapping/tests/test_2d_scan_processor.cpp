#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "rico_gmapping/rico_gmapping.hpp"
#include <vector>

/**
* @brief: brief
* @param: const 
* @return: void
*/

sensor_msgs::LaserScan::ConstPtr map_scan; 
void map_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    map_scan = msg; 
}

sensor_msgs::LaserScan::ConstPtr base_scan; 
void base_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    base_scan = msg; 
    if (map_scan){
        ROS_INFO_STREAM("Received map scan, processing base scan.");
    }
}

int main(int argc, char**argv){
    auto node_name = "test_2d_scan_processor";
    ros::init(argc, argv, node_name); 
    ros::NodeHandle nh("~"); 
    ROS_INFO_STREAM("Started "<<node_name); 

    auto map_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/map_scan", 1, map_scan_cb); 
    auto base_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_link_scan", 1, base_scan_cb); 
    ros::spin(); 
}
