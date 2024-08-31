#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // PointXYZRGB�͵���

    pcl::PLYReader reader;
    reader.read("E:\\seg_leave\\0718_WL82_2 - Cloud - Cloud.ply", *cloud);


    // ���������˲�
    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud(cloud);
    //sor.setLeafSize(0.005f, 0.005f, 0.005f);
    //sor.filter(*cloud);


    // ����һ���յ�kd-tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    // ����һ��normal����
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator; // �������߹��ƶ���
    normal_estimator.setSearchMethod(tree); // ������������
    normal_estimator.setInputCloud(cloud); // ���÷��߹��ƶ�������㼯
    normal_estimator.setRadiusSearch(0.005); // ʹ�ð뾶�ڲ�ѯ����Χ2���׷�Χ�ڵ�������Ԫ��
    normal_estimator.compute(*cloud_normals); // ���㲢���������

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg; // �������������ָ����
    reg.setMinClusterSize(350); // ����һ��������Ҫ����С����������С����ֵ�Ľ����������
    reg.setMaxClusterSize(100000); //����һ��������Ҫ�������������������ֵ�Ľ����������
    reg.setSearchMethod(tree); // ������������
    reg.setResidualThreshold(0.05); // ���������Ľ��ڵ���Ŀ

    reg.setInputCloud(cloud); // ����������� 
    reg.setInputNormals(cloud_normals); // �����������

    reg.setSmoothnessThreshold(3 / 180.0 * M_PI); //����ƽ����ֵ
    reg.setCurvatureThreshold(0.5); //�������ʷ�ֵ


    // �����������������ָ��㷨�������ؾ�������
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters); // ��ȡ����Ľ�����ָ��������ڵ���������������

    //��������������ӻ�

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer3("Cloud Viewer"); // ����һ�����ӻ�����
    viewer3.showCloud(colored_cloud);
    while (!viewer3.wasStopped())
    {
    }


    // ����ÿ������
    for (size_t i = 0; i < clusters.size(); ++i) {
        // ����һ���µ�PointCloud����
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // ���ݾ�����������µ�PointCloud����
        for (const auto& index : clusters[i].indices) {
            cluster_cloud->points.push_back(cloud->points[index]);
        }

        // Ϊÿ�����ഴ��һ���ļ���
        std::string filename = "E:\\seg_leave\\0718_WL82_2 - Cloud - Cloud_" + std::to_string(i) + ".ply";

        // ʹ��PLYWriter�������Ϊ�ļ�
        pcl::PLYWriter writer;
        writer.write(filename, *cluster_cloud);
    }
}
