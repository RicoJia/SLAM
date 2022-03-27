#ifndef __ICP_SVD_HPP__
#define __ICP_SVD_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.hpp>
#include <pcl/kdtree/kdtree_flann.h>
/**
* @brief: Use the SVD method to solve the iterative-closest-point problem, which solves for the affine transformation to align the current scan with the last scan
* @ Notes: potential problems: 
*   1. Association might be wrong. 
* @param: last_scan
* @param: current_scan
* @return: transformation current_scan -> last scan
*/
#include <iostream> 
using std::cout; using std::endl; 
Eigen::Affine3d icp_svd(const std::vector<Eigen::Vector2d>& last_scan, const std::vector<Eigen::Vector2d>&current_scan, unsigned int num_iterations = 3){

    // Build KD Tree with last_scan points [x, y] in base_link frame 
    // there's pcl::PointXYZ as well
    pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
    for (const auto& s : last_scan){ 
       cloud->push_back({s[0], s[1]}); 
    }
    pcl::KdTreeFLANN<pcl::PointXY>* kd_tree = new pcl::KdTreeFLANN<pcl::PointXY>(); 
    kd_tree -> setInputCloud(cloud);
    auto current_scan_cp = current_scan; 
    // Weird: auto R = Eigen::Matrix2d::Identity() auto will not be compatible with many. 
    // So must write Eigen::Matrix2d
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    for (unsigned int j = 0; j < num_iterations; ++j) {
        // Find matched points
        std::vector<Eigen::Vector2d> matched_last_scan;
        matched_last_scan.reserve(last_scan.size());
        for (unsigned int i = 0; i < current_scan.size(); ++i){
            auto& s = current_scan_cp.at(i); 
            s = R * current_scan.at(i) + t; 

            // we just want 1 neighbor
            int k = 1;
            std::vector<int> index(1);
            std::vector<float> squared_dist(1);
            kd_tree->nearestKSearch({s[0], s[1]}, k, index, squared_dist);
            matched_last_scan.emplace_back(last_scan.at(index[0]));
        }
        
        // Get miu
        Eigen::Vector2d mu_x(0.0, 0.0);
        Eigen::Vector2d mu_y(0.0, 0.0);
        for (int i = 0; i < matched_last_scan.size(); ++i){
            mu_x += matched_last_scan.at(i);
            mu_y += current_scan_cp.at(i);
        }

        mu_x /= matched_last_scan.size();
        mu_y /= current_scan_cp.size();

        // Get R, t.
        Eigen::Matrix2d H;
        for (int i = 0; i < matched_last_scan.size(); ++i){
            H += (matched_last_scan.at(i) - mu_x) * ((current_scan_cp.at(i) - mu_y).transpose());
        }
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // remember this is 2d
        R = svd.matrixU() * (svd.matrixV().transpose());
        for (unsigned int i = 0; i < 2; ++i) {
           R.col(i).normalize(); 
        }
        t  = mu_x - R * mu_y;
    }

    delete kd_tree; 

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(t[0], t[1], 0.0);
    transform.linear().block<2,2>(0,0) = R; 

    cout<<transform.matrix()<<endl;
    
    return transform;
}

#endif /* end of include guard: __ICP_SVN_HPP__ */
