#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_space.h>

// 定义全局变量，表示法向量滤波的参数
const float normal_threshold = 0.8; // 法向量滤波的阈值

// 欧几里得聚类
void euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02); // 两点之间的最大距离
    ec.setMinClusterSize(100); // 最小聚类尺寸
    ec.setMaxClusterSize(25000); // 最大聚类尺寸
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

// 法向量滤波
void normalFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud) {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(50); // 设置用于计算法向量的邻域点数
    ne.compute(*normals);

    pcl::NormalSpaceSampling<pcl::PointXYZRGB, pcl::Normal> nss;
    nss.setInputCloud(cloud);
    nss.setNormals(normals);
    nss.setBins(100, 100, 100); // 设置正态空间采样的网格尺寸
    nss.setSeed(0); // 设置随机种子
    nss.setSample(static_cast<unsigned int>(cloud->size() * 0.05)); // 设置采样点数

    pcl::PointIndices sampled_indices;
    nss.filter(sampled_indices.indices);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(std
        ::make_shared<const pcl::PointIndices>(sampled_indices));
    extract.setNegative(false);
    extract.filter(*filtered_cloud);
}

int main() {
    // 读取输入点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("p21_ds_rs_hsv.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // 使用欧几里得聚类进行分割
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud, cluster_indices);

    // 提取最大的聚类
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, cluster_indices[0], *largest_cluster);

    // 使用法向量滤波去除非植物表面的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    normalFilter(largest_cluster, filtered_cloud);

    // 保存结果
    pcl::io::savePCDFileBinary("p21_ds_rs_hsv_fil.pcd", *filtered_cloud);

    return 0;
}
