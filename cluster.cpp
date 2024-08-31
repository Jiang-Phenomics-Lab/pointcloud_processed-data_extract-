#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

void euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.2); // 两点之间的最大距离
    ec.setMinClusterSize(50); // 最小聚类尺寸
    ec.setMaxClusterSize(250); // 最大聚类尺寸
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int main() {
    // 读取输入点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("segmented_cloud2_ds.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // 使用欧几里得聚类进行分割
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud, cluster_indices);


    // 将每个聚类保存到不同的文件
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, cluster_indices[i], *cluster);
        std::stringstream ss;
        ss << "segmented_cloud2_ds_" << i << ".ply";
        pcl::io::savePLYFileBinary(ss.str(), *cluster);
    }

    return 0;
}
