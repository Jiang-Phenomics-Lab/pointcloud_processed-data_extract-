#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

int main() {
    // ��ȡ��������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
    pcl::PLYReader reader;
    reader.read(".\\0703_WL82\\0703_WL82_6.ply", *cloud);
    //if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1.ply", *cloud) == -1) {
    //    // ����ͳ���˲�������
    //    PCL_ERROR("Couldn't read file input.pcd \n");
    //    return (-1);
    //}
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);

    // �����˲�������
    sor.setMeanK(20); // ����������Ĭ��50
    sor.setStddevMulThresh(5); // ��׼�����ֵ
    
    // ִ���˲�����
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);

    pcl::PLYWriter writer;
    writer.write(".\\0703_WL82\\0703_WL82_6_f.ply", *cloud_filtered);

    // ����˲���ĵ�������
    //pcl::io::savePLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1_sf.ply", *cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->size() << " data points." << std::endl;

    return 0;
}
