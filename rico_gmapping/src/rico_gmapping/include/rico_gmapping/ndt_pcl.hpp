#ifndef __NDT_PCL_HPP__
#define __NDT_PCL_HPP__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

Eigen::Affine3d ndt_pcl(const std::vector<Eigen::Vector2d>& last_scan, const std::vector<Eigen::Vector2d>&current_scan){
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

    // Filtering input scan to roughly 10% of original size to increase speed of registration. This is just a uniform distribution downsampler. 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    // approximate_voxel_filter.setInputCloud (input_cloud);
    // approximate_voxel_filter.filter (*filtered_cloud);
    
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (0.1);

    ndt.setMaximumIterations (100);
    ndt.setInputSource (cloud_current);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (cloud_last);

    Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity(); 

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess.cast<float>());

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()<< " score: " << ndt.getFitnessScore () << std::endl;

    Eigen::Affine3d ret;
    ret.matrix() = ndt.getFinalTransformation().cast<double> ();
    //Get the final transformation matrix estimated by the registration method.
    std::cout <<ret.matrix()<< std::endl;
    return ret; 
}
    

#endif /* end of include guard: __NDT_PCL_HPP__ */
