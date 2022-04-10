#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// you need this for converting eigen to tf eigen 
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include "rico_gmapping/rico_gmapping.hpp"
#include "rico_gmapping/icp_svd.hpp"
#include "rico_gmapping/icp_gn.hpp"
#include "rico_gmapping/icp_pcl.hpp"
#include "rico_gmapping/ndt_pcl.hpp"
#include <vector>
#include <iostream>
#include <cmath>
using std::cout; using std::endl; 

std::vector<Eigen::Vector2d> last_scan; 
tf::Transform transform; 

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

void base_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Eigen::Affine3d t_map_base_scan; 
    if (!last_scan.empty()){ 
        ROS_INFO_STREAM("Received map scan, processing base scan.");

        std::vector<Eigen::Vector2d> current_scan; 
        build_valid_scan_vec(msg, current_scan); 

        // // svd & gn 
        // t_map_base_scan = icp_svd(last_scan, current_scan, 5);
        // t_map_base_scan = icp_gn(last_scan, current_scan, 20, t_map_base_scan); 
        // t_map_base_scan = icp_gn(last_scan, current_scan, 20); 
        t_map_base_scan = ndt_pcl(last_scan, current_scan); 
        // t_map_base_scan = icp_pcl(last_scan, current_scan); 
    }

    tf::transformEigenToTF(t_map_base_scan, transform); 
}

/**
* @ Show laser scans in tf properly
*  - Even though you want to "statically publish" a tf, you still need to periodically publish it.
*  - DO NOT select "unreliable" in laser scans
*/
int main(int argc, char**argv){
    auto node_name = "test_2d_scan_processor";
    ros::init(argc, argv, node_name); 
    ros::NodeHandle nh("~"); 
    ROS_INFO_STREAM("Started "<<node_name); 

    tf::Quaternion q; 
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    // publish tf 
    tf::TransformBroadcaster br; 

    auto map_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/map_scan", 1, map_scan_cb); 
    auto base_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, base_scan_cb); 

    ros::Rate r(1);
    while(ros::ok()){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_scan"));
        ROS_INFO_STREAM("Published TF map->base_scan"); 
        ros::spinOnce();
        r.sleep();
    }
}
