#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

void euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.2); // ����֮���������
    ec.setMinClusterSize(50); // ��С����ߴ�
    ec.setMaxClusterSize(250); // ������ߴ�
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int main() {
    // ��ȡ�������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("segmented_cloud2_ds.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // ʹ��ŷ����þ�����зָ�
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud, cluster_indices);


    // ��ÿ�����ౣ�浽��ͬ���ļ�
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, cluster_indices[i], *cluster);
        std::stringstream ss;
        ss << "segmented_cloud2_ds_" << i << ".ply";
        pcl::io::savePLYFileBinary(ss.str(), *cluster);
    }

    return 0;
}
