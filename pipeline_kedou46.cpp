#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>


// Function to convert RGB to HSV
void rgbToHsv(float r, float g, float b, float& h, float& s, float& v) {
    r = r / 255.0;
    g = g / 255.0;
    b = b / 255.0;
    float max = std::max(r, std::max(g, b)), min = std::min(r, std::min(g, b));
    v = max;

    float diff = max - min;
    s = max == 0 ? 0 : diff / max;

    if (max == min) {
        h = 0; // achromatic
    }
    else {
        if (max == r) {
            h = (g - b) / diff + (g < b ? 6 : 0);
        }
        else if (max == g) {
            h = (b - r) / diff + 2;
        }
        else if (max == b) {
            h = (r - g) / diff + 4;
        }
        h /= 6;
    }
    h *= 360; // Convert to degrees
}

void euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.015); // ����ʱ����֮���������//0.08before,0.05,0.025
    ec.setMinClusterSize(500); // �������С�ߴ�//500before
    ec.setMaxClusterSize(150000); // ��������ߴ�
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}



int main(int argc, char** argv) {
    // Load input point cloud
    //std::vector<std::string> patterns = { "jidou19","kedou35","zhongdou41","liaodou50","heihe43","kedou46","WL82" };
    std::vector<std::string> patterns = { "kedou46" };
    // ԭʼ�ļ������ļ���ǰ׺
    std::string prefix = "./0822/0822_";
    std::string prefix_after = "./0822_single/0822_";

    std::string original_suffix = "_ds_sf.ply";
    std::string new_suffix = "_hsv.ply";
    std::string _hsv = "_hsv";
    std::string origin = ".ply";
    std::string us = "_us_";

    std::string ds = "_ds.ply";
    std::string sf = "_sf.ply";

    // ����patterns�е�ÿ��ģʽ
    for (const auto& pattern : patterns) {
        // ����ԭʼ�ļ��������ļ���
        std::string ds_sf_file = prefix + pattern + original_suffix;
        std::string hsv_file = prefix + pattern + new_suffix;
        std::string hsv_file_name = prefix + pattern + _hsv;
        std::string origin_file = prefix + pattern + origin;
        std::string usfile = prefix_after + pattern + us;

        std::string ds_file = prefix + pattern + ds;
        std::string sf_file = prefix + pattern + original_suffix;

        typedef pcl::PointXYZRGB PointType;
        pcl::PointCloud<PointType>::Ptr cloud_ds(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_filtered_ds(new pcl::PointCloud<PointType>());

        // PLY��ȡ��
        pcl::PLYReader reader;

        // ��ȡPLY�ļ�
        reader.read(origin_file, *cloud_ds); // ȷ���ļ�·����ȷ

        std::cerr << "PointCloud before filtering: " << cloud_ds->size()
            << " data points." << std::endl;

        // �����˲���
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(cloud_ds);
        // �������ش�С
        sor.setLeafSize(0.008f, 0.008f, 0.008f);
        sor.filter(*cloud_filtered_ds);

        std::cerr << "PointCloud after filtering: " << cloud_filtered_ds->size()
            << " data points." << std::endl;

        // PLYд����
        pcl::PLYWriter writer;

        // д��PLY�ļ�
        writer.write(ds_file, *cloud_filtered_ds); // true ��ʾʹ�ö����Ƹ�ʽд��

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sf(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
        pcl::PLYReader reader_sf;
        reader_sf.read(ds_file, *cloud_sf);
        //if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1.ply", *cloud) == -1) {
        //    // ����ͳ���˲�������
        //    PCL_ERROR("Couldn't read file input.pcd \n");
        //    return (-1);
        //}
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_sf;
        sor_sf.setInputCloud(cloud_sf);

        // �����˲�������
        sor_sf.setMeanK(100); // ����������Ĭ��50
        sor_sf.setStddevMulThresh(1); // ��׼�����ֵ

        // ִ���˲�����
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sf(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor_sf.filter(*cloud_filtered_sf);

        pcl::PLYWriter writer_sf;
        writer_sf.write(sf_file, *cloud_filtered_sf);

        // ����˲���ĵ�������
        //pcl::io::savePLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1_sf.ply", *cloud_filtered);

        std::cout << "PointCloud after filtering: " << cloud_filtered_sf->size() << " data points." << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(ds_sf_file, *cloud) == -1) {
            PCL_ERROR("Couldn't read file input.pcd \n");
            return (-1);
        }

        // Filtered point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Filter points based on HSV values
        for (const auto& point : *cloud) {
            float h, s, v;
            rgbToHsv(point.r, point.g, point.b, h, s, v);

            // Check if the hue value is within the desired range (50-120)//(50,170),(0.25,),(0.1,0.7)
            if (h >= 40 && h <= 175 && s >= 0.1 && v <= 0.85 && v >= 0.02) {
                filtered_cloud->push_back(point);
            }
        }

        // Save the filtered point cloud
        pcl::io::savePLYFile<pcl::PointXYZRGB>(hsv_file, *filtered_cloud);

        std::cout << "hsv Filtered point cloud saved" << std::endl;


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(hsv_file, *cloud_cluster) == -1) {
            PCL_ERROR("Couldn't read input file!\n");
            return -1;
        }

        // ʹ��ŷ����þ�����зָ�
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanClustering(cloud_cluster, cluster_indices);

        // ��ÿ�����ౣ��Ϊ�����ĵ����ļ�
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud_cluster, cluster_indices[i], *cluster);
            std::stringstream ss;
            ss << hsv_file_name << "_" << i << ".ply";
            pcl::io::savePLYFileBinary(ss.str(), *cluster);
        }



        for (int i = 0; i < cluster_indices.size(); i++) {
            // Ӳ��������ļ�·��
            std::cout << "Processing cluster: " << i << std::endl;
            std::ostringstream oss;
            oss << hsv_file_name << "_" << i << ".ply";
            std::string path = oss.str();

            std::string origin = origin_file;
            std::string hsv_cluster = path;

            // ��ȡ�����ļ�
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bbox(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(hsv_cluster, *cloud_bbox) == -1) {
                std::cerr << "Error reading PLY file " << hsv_cluster << std::endl;
                return -1;
            }
            std::cout << "Loaded bbox point cloud from " << hsv_cluster << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(origin, *cloud_origin) == -1) {
                std::cerr << "Error reading PLY file " << origin << std::endl;
                return -1;
            }
            std::cout << "Loaded origin point cloud from " << origin << std::endl;

            // ����߽����С���������
            pcl::PointXYZRGB minPt, maxPt;
            pcl::getMinMax3D(*cloud_bbox, minPt, maxPt);

            // ����߽�����С���������
            float min_x = minPt.x - 0.04;
            float min_y = minPt.y - 0.04;
            float min_z = minPt.z - 0.2;
            float max_x = maxPt.x + 0.04;
            float max_y = maxPt.y + 0.04;
            float max_z = maxPt.z + 0.2;

            // �����µĵ��ƶ������ڴ�ŷָ���ĵ�
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // ����ԭʼ���ƣ����ڱ߽���ڵĵ�����µĵ�����
            for (const pcl::PointXYZRGB& point : cloud_origin->points) {
                float h, s, v;
                rgbToHsv(point.r, point.g, point.b, h, s, v);

                if (point.x >= min_x && point.x <= max_x &&
                    point.y >= min_y && point.y <= max_y &&
                    point.z >= min_z && point.z <= max_z &&
                    (h >= 20 && h <= 160 && s >= 0.05 && v <= 0.95 && v >= 0.05))
                    //(h >= 20 && h <= 180 && s >= 0.01 && v <= 0.99 && v >= 0.01))
/*                if (point.x >= min_x && point.x <= max_x &&
                        point.y >= min_y && point.y <= max_y &&
                        point.z >= min_z && point.z <= max_z
                        ) */ {
                    segmented_cloud->points.push_back(point);
                }
            }
            std::cout << "Segmented cloud points count: " << segmented_cloud->points.size() << std::endl;
            //h >= 60 && h <= 150 && s >= 0.15 && v <= 0.85 && v >= 0.1

            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> s;
            s.setInputCloud(segmented_cloud);

            //// �����˲�������
            s.setMeanK(50); // ����������Ĭ��50
            s.setStddevMulThresh(4); // ��׼�����ֵ3

            // ִ���˲�����
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
            s.filter(*cloud_filtered_seg);

            // �����µ��ƵĿ�Ⱥ͸߶�
            //segmented_cloud->width = segmented_cloud->points.size();
            //segmented_cloud->height = 1;

            // ��ӡ�ָ����Ƶ���Ϣ
            std::cout << "Filtered segmented cloud points count: " << cloud_filtered_seg->points.size() << std::endl;
            std::ostringstream oss1;

            oss1 << usfile << i << ".ply";
            std::string final_cluster = oss1.str();
            // ����ָ��ĵ���
            if (cloud_filtered_seg->points.size() >= 10000) {
                pcl::io::savePLYFile(final_cluster, *cloud_filtered_seg);
            }
            //if (segmented_cloud->points.size() >= 10000) {
            //    pcl::io::savePLYFile(final_cluster, *segmented_cloud);
            //}
        }

    }


    return 0;
}
