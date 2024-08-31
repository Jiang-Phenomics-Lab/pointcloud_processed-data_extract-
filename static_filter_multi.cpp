#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

int main() {
    for (int i = 0; i < 6; i++) {
        // ��ȡ��������
        std::ostringstream oss;
        oss << "./0529/0529_jidou19_us_" << i << "_rs.ply";
        std::string path = oss.str();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
        pcl::io::loadPLYFile<pcl::PointXYZRGB>(path, *cloud);
        // ����ͳ���˲�������
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);

        // �����˲�������
        sor.setMeanK(0.6); // ����������Ĭ��50
        sor.setStddevMulThresh(1); // ��׼�����ֵ

        // ִ���˲�����
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*cloud_filtered);
        std::ostringstream oss1;
        oss1 << "./0529/0529_jidou19_sf_" << i << ".ply";
        std::string path1 = oss1.str();
        // ����˲���ĵ�������
        pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(path1, *cloud_filtered);

        std::cout << "PointCloud after filtering: " << cloud_filtered->size() << " data points." << std::endl;
    }
    return 0;
}
