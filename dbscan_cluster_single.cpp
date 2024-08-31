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
    ec.setClusterTolerance(0.01); // ����ʱ����֮���������//0.08before
    ec.setMinClusterSize(50); // �������С�ߴ�//500before
    ec.setMaxClusterSize(250000); // ��������ߴ�
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int main() {
    // ��ȡ�������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_liaodou50_ds_sf_hsv.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // ʹ��ŷ����þ�����зָ�
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud, cluster_indices);

    // ��ÿ�����ౣ��Ϊ�����ĵ����ļ�
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, cluster_indices[i], *cluster);
        std::stringstream ss;
        ss << "./final_dataset/0520_liaodou50_ds_sf_hsv_" << i << ".ply";
        pcl::io::savePLYFileBinary(ss.str(), *cluster);
    }

    return 0;
}
