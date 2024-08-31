#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

int main() {
    // 输入点云路径
    for (int i = 0; i < 6; i++) {
        std::string prefix = "./0529/0529_";
        std::string cl = "kedou46";
        std::string us = "_us_";
        std::string _ply = ".ply";
        std::string rs = "_rs_";
        std::string input_cloud_path = prefix + cl + us + std::to_string(i) + _ply;
        // 输出点云路径
        std::string output_cloud_path = prefix + cl + rs + std::to_string(i) + _ply;

        // Load input point cloud data
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_cloud_path, *cloud) == -1) {
            PCL_ERROR("Couldn't read input file!\n");
            return -1;
        }

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Set the segmentation parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.015);

        // Segment the largest planar component from the input cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Extract the inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground);

        // Remove ground points from the original cloud
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*cloud_without_ground);

        // Save the ground removed point cloud
        pcl::io::savePLYFileBinary(output_cloud_path, *cloud_without_ground);

        // Visualization
        pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
        viewer.setBackgroundColor(0, 0, 0);
        viewer.addPointCloud(cloud, "original_cloud");
        viewer.addPointCloud(ground, "ground_points");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "ground_points");
        viewer.spin();
    }

    return 0;
}
