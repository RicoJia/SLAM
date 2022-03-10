#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "rico_gmapping/rico_gmapping.hpp"
#include <vector>

/**
* @brief: random function to fill in map info
* @param: nav_msgs::OccupancyGrid& 
* @return: void
*/
void fill_in_map(nav_msgs::OccupancyGrid& map)
{
    map.header.stamp = ros::Time::now();
    map.header.frame_id = "/map";
    map.info.resolution = 0.5; // m/cell
    map.info.height = 100;   // # cells
    map.info.width = 100;
    map.info.origin.orientation.w = 1.0;
    map.info.map_load_time = ros::Time::now();
        map.info.origin.position.y = 0.0;
    map.info.origin.position.x = 0.0; 
    // really, this is an int8[], so just 1D array
    map.data.assign(map.info.height * map.info.width, 0);
}

/**
 * Theory
 *  1. rviz error: Error subscribing: Character [ ] at element [18] is not valid in Graph Resource Name [/rico_gmapping/map ].  Valid characters are a-z, A-Z, 0-9, / and _., this means the map topic name you put into rviz is not correct, maybe have an extra space or something
*/
int main(int argc, char**argv)
{
    /**
    * @brief: ROS Node 
    */
    // node name is hello
    ros::init(argc, argv, "rico_gmapping"); 
    ros::NodeHandle nh("~"); 
    // https://github.com/msc9533/ros-local-map-publisher/blob/master/src/local_map_builder/src/local_map.cpp
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ros::Rate r(50);
    nav_msgs::OccupancyGrid map;
    while(ros::ok()){
        fill_in_map(map);
        map_pub.publish(map); 
        ros::spinOnce();
        r.sleep();
    }
}
