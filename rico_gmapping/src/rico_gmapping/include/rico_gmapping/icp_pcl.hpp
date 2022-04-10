#ifndef __ICP_PCL_HPP__
#define __ICP_PCL_HPP__
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/videoio.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

Eigen::Affine3d icp_pcl(const std::vector<Eigen::Vector2d>& last_scan, const std::vector<Eigen::Vector2d>&current_scan){
    // create source and target pt clounds
    // HAVE TO USE PointXYZ not PointXY
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < std::min(last_scan.size(), current_scan.size()); ++i) {
        cloud_last->push_back({last_scan.at(i)[0], last_scan.at(i)[1], 0.0});
        cloud_current->push_back({current_scan.at(i)[0], current_scan.at(i)[1], 0.0});
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_current);
    icp.setInputTarget(cloud_last);

    //Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm
    pcl::PointCloud<pcl::PointXYZ> Final;

    //Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
    icp.align(Final);

    //Return the state of convergence after the last align run.
    //If the two PointClouds align correctly then icp.hasConverged() = 1 (true).
    std::cout << "has converged: " << icp.hasConverged() <<std::endl;

    //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
    std::cout << "score: " <<icp.getFitnessScore() << std::endl;
    //
    Eigen::Affine3d ret; 
    ret.matrix() = icp.getFinalTransformation().cast<double> ();
    //Get the final transformation matrix estimated by the registration method.
    std::cout <<ret.matrix()<< std::endl;
    return ret;

}



#endif /* end of include guard: __ICP_PCL_HPP__ */
