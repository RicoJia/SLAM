#ifndef __ICP_GN_HPP__
#define __ICP_GN_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <pcl/kdtree/kdtree_flann.h>

/**
* @brief: Use the Gauss Newton method to solve the iterative-closest-point problem, which solves for the affine transformation to align the current scan with the last scan
* @param: last_scan
* @param: current_scan
* @param: initial_guess: if not supplied, it will be identity
* @return: Eigen::Affine3d
*/
Eigen::Affine3d icp_gn(const std::vector<Eigen::Vector2d>& last_scan, const std::vector<Eigen::Vector2d>&current_scan, unsigned int num_iterations = 3, Eigen::Affine3d initial_guess=Eigen::Affine3d::Identity())
{
   // Build 2D KD Tree 
   // Match current points to the closest point in the KD Tree
    pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
    for (const auto& s : last_scan){ 
        cloud->push_back({s[0], s[1]}); 
    }
    pcl::KdTreeFLANN<pcl::PointXY>* kd_tree = new pcl::KdTreeFLANN<pcl::PointXY>(); 
    kd_tree -> setInputCloud(cloud);
    auto current_scan_updated = current_scan; 

    Eigen::Matrix2d R = initial_guess.matrix().block(0,0,2,2);
    Eigen::Vector2d t = initial_guess.matrix().block(0,3,2,1);
    for (unsigned int j = 0; j < num_iterations; ++j){
        // Find matched points
        std::vector<Eigen::Vector2d> matched_last_scan;
        matched_last_scan.reserve(last_scan.size());
        // square mean error
        double rmse = 0; 
        for (unsigned int i = 0; i < current_scan.size(); ++i){
            auto& s = current_scan_updated.at(i); 
            s = R * current_scan.at(i) + t; 

            // we just want 1 neighbor
            int k = 1;
            std::vector<int> index(1);
            std::vector<float> squared_dist(1);
            kd_tree->nearestKSearch({s[0], s[1]}, k, index, squared_dist);
            matched_last_scan.emplace_back(last_scan.at(index[0]));
            rmse += (last_scan.at(index[0]) - s).norm(); 
        }
        std::cout<<__FUNCTION__<<": rmse: "<<rmse<<std::endl;

        Eigen::Matrix<double, 6, 6> H_sum = Eigen::Matrix<double, 6, 6>::Zero();  
        Eigen::Matrix<double, 6, 1> b_sum = Eigen::Matrix<double, 6, 1>::Zero();  
        for (int i = 0; i < matched_last_scan.size(); ++i){
            Eigen::Matrix3d R_3d = Eigen::Matrix3d::Identity();
            R_3d.block<2,2>(0,0) = R;
            Eigen::Vector3d y_3d = Eigen::Vector3d::Zero();
            y_3d.block<2,1>(0,0) = current_scan_updated.at(i);

            // get J = -[I hat(-R*yi)]
            Eigen::Matrix<double, 3, 6> J; 
            J.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); 
            J.block<3,3>(0,3) = -R_3d * Sophus::SO3<double>::hat(y_3d); 
            J = -J; 

            // get e = xi - (R*yi + t) 
            // e -> 3D
            Eigen::Vector3d e_3d = Eigen::Vector3d::Zero(); 
            e_3d.block<2,1>(0,0) = matched_last_scan.at(i) - current_scan_updated.at(i); 
            // H, b
            Eigen::Matrix<double, 6, 6> H = J.transpose() * J; 
            Eigen::Matrix<double, 6, 1> b = -J.transpose() * e_3d; 
            H_sum += H; 
            b_sum += b;
        }

        // epsilon
        if (H_sum.determinant() == 0){
            std::cout<<__FUNCTION__<<"H sum is not full rank. Aborted"<<std::endl;
            break;
        }
        // Cholesky solver, an alternative to b_sum
        // Eigen::Matrix<double,6,1> epsilon = H_sum.inverse() * b_sum; 
        Eigen::Matrix<double,6,1> epsilon = H_sum.ldlt().solve(b_sum); 
        // update R, t
        t += epsilon.block<2,1>(0,0); 
        auto delta_R = Sophus::SO2d::exp(epsilon(5)); 
        R = R * delta_R.matrix(); // right disturbance
    }

    delete kd_tree; 

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(t[0], t[1], 0.0);
    transform.linear().block<2,2>(0,0) = R; 
    cout<<transform.matrix()<<endl;
    return transform;
}


#endif /* end of include guard: __ICP_GN_HPP__ */
