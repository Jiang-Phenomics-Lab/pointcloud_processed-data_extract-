#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

void densityBasedSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    // Euclidean clustering based on density
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.25); // Cluster tolerance (minimum distance between clusters)
    ec.setMinClusterSize(50);   // Minimum cluster size (points)
    ec.setMaxClusterSize(1000); // Maximum cluster size (points)
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int main() {
    // Load point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("0603_zhongdou41_3_label1.ply", *cloud) == -1) {
        std::cerr << "Couldn't read input file!" << std::endl;
        return -1;
    }

    // Perform density-based segmentation
    std::vector<pcl::PointIndices> cluster_indices;
    densityBasedSegmentation(cloud, cluster_indices);

    // Save segmented clusters to separate files
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, cluster_indices[i], *cluster);
        std::stringstream ss;
        ss << "0603_zhongdou41_3_label1_" << i << ".ply";
        pcl::io::savePLYFileBinary(ss.str(), *cluster);
    }

    return 0;
}
