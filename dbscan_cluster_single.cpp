#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h> 

void euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 聚类时两点之间的最大距离//0.08before
    ec.setMinClusterSize(50); // 聚类的最小尺寸//500before
    ec.setMaxClusterSize(250000); // 聚类的最大尺寸
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int main() {
    // 读取输入点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_liaodou50_ds_sf_hsv.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // 使用欧几里得聚类进行分割
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud, cluster_indices);

    // 将每个聚类保存为单独的点云文件
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, cluster_indices[i], *cluster);
        std::stringstream ss;
        ss << "./final_dataset/0520_liaodou50_ds_sf_hsv_" << i << ".ply";
        pcl::io::savePLYFileBinary(ss.str(), *cluster);
    }

    return 0;
}
