#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    //  ‰»Îµ„‘∆¬∑æ∂
    //std::string input_cloud_path = "7pn_sf_hf.pcd";
    std::string input_cloud_path = "E:\\seg_leave\\lccp_segment_1_transformed.ply";

    // Load input point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_cloud_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud(cloud, "original_cloud");

    // Add XYZ axes
    viewer.addCoordinateSystem(1.0, "global");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}
