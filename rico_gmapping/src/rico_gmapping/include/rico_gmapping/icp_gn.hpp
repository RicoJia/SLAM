#ifndef __ICP_GN_HPP__
#define __ICP_GN_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/src/Geometry/Transform.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.hpp>
#include <pcl/kdtree/kdtree_flann.h>

/**
* @brief: Use the Gauss Newton method to solve the iterative-closest-point problem, which solves for the affine transformation to align the current scan with the last scan
* @param: last_scan
* @param: current_scan
* @param: initial_guess: if not supplied, it will be identity
* @return: Eigen::Affine3d
*/
Eigen::Affine3d icp_gn(const std::vector<Eigen::Vector2d>& last_scan, const std::vector<Eigen::Vector2d>&current_scan, Eigen::Affine3d initial_guess=Eigen::Affine3d::Identity(), unsigned int num_iterations = 3)
{
   // Build KD Tree 
   // Match current points to the closest point in the KD Tree
    pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
    for (const auto& s : last_scan){ 
    cloud->push_back({s[0], s[1]}); 
    }
    pcl::KdTreeFLANN<pcl::PointXY>* kd_tree = new pcl::KdTreeFLANN<pcl::PointXY>(); 
    kd_tree -> setInputCloud(cloud);
    auto current_scan_cp = current_scan; 

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    for (unsigned int j = 0; j < num_iterations; ++j) {}

}


#endif /* end of include guard: __ICP_GN_HPP__ */
