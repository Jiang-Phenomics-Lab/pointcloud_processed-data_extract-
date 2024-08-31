#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

int main() {
    // ��ȡ��������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("6p8p_nerf_sf_hf_vf.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // ���㷨����
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(30);
    normal_estimator.compute(*normals);

    // ���ڷ�������������������
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(5000); // ��С����ߴ�
    reg.setMaxClusterSize(100000000); // ������ߴ�
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30000); // ���������
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // ƽ����ֵ
    reg.setCurvatureThreshold(1.0); // ������ֵ

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // �������Ϊ���࣬ÿ�����һ��ֲ��
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> plant_clusters(6);
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (clusters[i].indices.size() > 1000 && clusters[i].indices.size() < 5000) {
            if (!plant_clusters[0])
                plant_clusters[0].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud, clusters[i], *plant_clusters[0]);
        }
        else if (clusters[i].indices.size() > 5000 && clusters[i].indices.size() < 10000) {
            if (!plant_clusters[1])
                plant_clusters[1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud, clusters[i], *plant_clusters[1]);
        }
        else if (clusters[i].indices.size() > 10000 && clusters[i].indices.size() < 15000) {
            if (!plant_clusters[2])
                plant_clusters[2].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud, clusters[i], *plant_clusters[2]);
        }
        else if (clusters[i].indices.size() > 15000) {
            if (!plant_clusters[3])
                plant_clusters[3].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud, clusters[i], *plant_clusters[3]);
        }
    }

    // ��ÿ����𱣴浽�������ļ�
    for (size_t i = 0; i < plant_clusters.size(); ++i) {
        if (plant_clusters[i]) {
            std::stringstream ss;
            ss << "nerf_plant_" << i << ".pcd";
            pcl::io::savePCDFileBinary(ss.str(), *plant_clusters[i]);
        }
    }

    return 0;
}
