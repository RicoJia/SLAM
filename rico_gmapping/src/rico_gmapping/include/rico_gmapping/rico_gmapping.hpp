#ifndef __RICO_GMAPPING_HPP__
#define __RICO_GMAPPING_HPP__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class RicoGmapping{
    public:
        RicoGmapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        void start_live_slam();
    private: 
        ros::Publisher map_pub_; 
        nav_msgs::OccupancyGrid map_;
};


#endif /* end of include guard: __RICO_GMAPPING_HPP__ */
