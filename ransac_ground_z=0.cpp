#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    // �������·��
    std::string input_cloud_path = "7pn_sf_hf.pcd";

    // Load input point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_cloud_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read input file!\n");
        return -1;
    }

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Set the segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return -1;
    }

    // ��ȡ���ƽ��ķ�������ƽ���ϵ�һ����
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    if (plane_normal.z() < 0) plane_normal = -plane_normal; // ȷ��������Z����Ϊ��

    // ������Z��ƽ�Ƶľ��룬ʹ�õ���λ��Z���ԭ��
    float d = coefficients->values[3];
    Eigen::Vector3f translation_vector(0, 0, -d / plane_normal.z());

    // ����ƽ�Ʊ任����
    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    transform_translation.translation() = translation_vector;

    // Ӧ��ƽ�Ʊ任������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_translated(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *cloud_translated, transform_translation);

    // ������ת��ͽǶȣ�ʹ�÷��������뵽Z��
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f rotation_axis = plane_normal.cross(z_axis);
    rotation_axis.normalize();
    float angle = acos(plane_normal.dot(z_axis));

    // ������ת�任����
    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    transform_rotation.rotate(Eigen::AngleAxisf(angle, rotation_axis));

    // Ӧ����ת�任������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_translated, *cloud_rotated, transform_rotation);

    // Visualization
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud(cloud_rotated, "rotated_cloud");

    // Add XYZ axes
    viewer.addCoordinateSystem(1.0, "global");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    pcl::io::savePCDFileBinary("7pn_sf_hf_rs.pcd", *cloud_rotated);
    std::cout << "Saved transformed and filtered cloud to 'transformed_and_filtered_cloud.pcd'." << std::endl;

    return 0;
}

