#include "rico_gmapping/rico_gmapping.hpp"

int main(int argc, char**argv)
{
    /**
    * @brief: ROS Node 
    */
    ros::init(argc, argv, "rico_gmapping"); 
    ros::NodeHandle nh("~"); 
    // https://github.com/msc9533/ros-local-map-publisher/blob/master/src/local_map_builder/src/local_map.cpp
     = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ros::Rate r(50);
    
    while(ros::ok()){
        fill_in_map(map);
        map_pub.publish(map); 
        ros::spinOnce();
        r.sleep();
    }
}

