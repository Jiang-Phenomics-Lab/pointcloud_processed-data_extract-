#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
int main() {
    // ��ȡ��������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("7pn_0_hf_tran.pcd", *cloud);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("7pn_0510_6_hf.ply", *cloud);
    // ������С�����߶�
    float min_height = -0.03; // ��С�߶�before0.1
    float max_height = 1; // ���߶�

    // ����һ�������˲���
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr height_condition(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    height_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, min_height))); // ��С�߶�����
    height_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, max_height))); // ���߶�����

    // ���������˲�������
    pcl::ConditionalRemoval<pcl::PointXYZRGB> height_filter;
    height_filter.setInputCloud(cloud);
    height_filter.setCondition(height_condition);
    //height_filter.setKeepOrganized(true); // ���ֵ��Ƶ�������

    // ִ���˲�����
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    height_filter.filter(*cloud_filtered);

    // Visualization
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud(cloud_filtered, "cloud_filtered");

    // Add XYZ axes
    viewer.addCoordinateSystem(1.0, "global");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    // ����˲���ĵ�������
    //pcl::io::savePCDFile<pcl::PointXYZRGB>("7pn_0_hf_tran_hftest.pcd", *cloud_filtered);
    pcl::io::savePLYFile<pcl::PointXYZRGB>("7pn_0510_6_hf.ply", *cloud_filtered);
    std::cout << "PointCloud after height filtering: " << cloud_filtered->size() << " data points." << std::endl;

    return 0;
}
