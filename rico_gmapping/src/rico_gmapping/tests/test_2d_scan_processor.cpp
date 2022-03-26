#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// you need this for converting eigen to tf eigen 
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include "rico_gmapping/rico_gmapping.hpp"
#include "rico_gmapping/icp_svd.hpp"
#include <vector>
#include <iostream>
#include <cmath>
using std::cout; using std::endl; 

std::vector<Eigen::Vector2d> last_scan; 

void build_valid_scan_vec(const sensor_msgs::LaserScan::ConstPtr& msg, std::vector<Eigen::Vector2d>& scan){
    double angle = 0.0; 
    for (const auto& s: msg->ranges){
        if (msg->range_min < s && s < msg->range_max){
           scan.emplace_back(s * std::cos(angle), s * std::sin(angle)) ; 
        }
        angle += msg -> angle_increment; 
    } 
}

void map_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    build_valid_scan_vec(msg, last_scan); 
}

void base_link_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Eigen::Affine3d t_map_base_scan; 
    if (!last_scan.empty()){ 
        ROS_INFO_STREAM("Received map scan, processing base scan.");

        std::vector<Eigen::Vector2d> current_scan; 
        build_valid_scan_vec(msg, current_scan); 

        t_map_base_scan = icp_svd(last_scan, current_scan, 5);
    }

    // publish tf 
    static tf::TransformBroadcaster br; 
    tf::Transform transform; 
    tf::transformEigenToTF(t_map_base_scan, transform); 
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_scan"));
    ROS_INFO_STREAM("Published TF map->base_scan"); 
}

int main(int argc, char**argv){
    auto node_name = "test_2d_scan_processor";
    ros::init(argc, argv, node_name); 
    ros::NodeHandle nh("~"); 
    ROS_INFO_STREAM("Started "<<node_name); 

    auto map_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/map_scan", 1, map_scan_cb); 
    auto base_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_link_scan", 1, base_link_scan_cb); 
    ros::spin(); 
}
