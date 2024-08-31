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
    ec.setClusterTolerance(0.015); // 聚类时两点之间的最大距离//0.08before,0.05,0.025
    ec.setMinClusterSize(500); // 聚类的最小尺寸//500before
    ec.setMaxClusterSize(150000); // 聚类的最大尺寸
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}



int main(int argc, char** argv) {
    // Load input point cloud
    //std::vector<std::string> patterns = { "jidou19","kedou35","zhongdou41","liaodou50","heihe43","kedou46","WL82" };
    std::vector<std::string> patterns = { "kedou46" };
    // 原始文件和新文件的前缀
    std::string prefix = "./0822/0822_";
    std::string prefix_after = "./0822_single/0822_";

    std::string original_suffix = "_ds_sf.ply";
    std::string new_suffix = "_hsv.ply";
    std::string _hsv = "_hsv";
    std::string origin = ".ply";
    std::string us = "_us_";

    std::string ds = "_ds.ply";
    std::string sf = "_sf.ply";

    // 遍历patterns中的每个模式
    for (const auto& pattern : patterns) {
        // 构建原始文件名和新文件名
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

        // PLY读取器
        pcl::PLYReader reader;

        // 读取PLY文件
        reader.read(origin_file, *cloud_ds); // 确保文件路径正确

        std::cerr << "PointCloud before filtering: " << cloud_ds->size()
            << " data points." << std::endl;

        // 体素滤波器
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(cloud_ds);
        // 设置体素大小
        sor.setLeafSize(0.008f, 0.008f, 0.008f);
        sor.filter(*cloud_filtered_ds);

        std::cerr << "PointCloud after filtering: " << cloud_filtered_ds->size()
            << " data points." << std::endl;

        // PLY写入器
        pcl::PLYWriter writer;

        // 写入PLY文件
        writer.write(ds_file, *cloud_filtered_ds); // true 表示使用二进制格式写入

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sf(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
        pcl::PLYReader reader_sf;
        reader_sf.read(ds_file, *cloud_sf);
        //if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1.ply", *cloud) == -1) {
        //    // 创建统计滤波器对象
        //    PCL_ERROR("Couldn't read file input.pcd \n");
        //    return (-1);
        //}
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_sf;
        sor_sf.setInputCloud(cloud_sf);

        // 设置滤波器参数
        sor_sf.setMeanK(100); // 邻域点的数量默认50
        sor_sf.setStddevMulThresh(1); // 标准差倍数阈值

        // 执行滤波操作
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sf(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor_sf.filter(*cloud_filtered_sf);

        pcl::PLYWriter writer_sf;
        writer_sf.write(sf_file, *cloud_filtered_sf);

        // 输出滤波后的点云数据
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

        // 使用欧几里得聚类进行分割
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanClustering(cloud_cluster, cluster_indices);

        // 将每个聚类保存为单独的点云文件
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud_cluster, cluster_indices[i], *cluster);
            std::stringstream ss;
            ss << hsv_file_name << "_" << i << ".ply";
            pcl::io::savePLYFileBinary(ss.str(), *cluster);
        }



        for (int i = 0; i < cluster_indices.size(); i++) {
            // 硬编码点云文件路径
            std::cout << "Processing cluster: " << i << std::endl;
            std::ostringstream oss;
            oss << hsv_file_name << "_" << i << ".ply";
            std::string path = oss.str();

            std::string origin = origin_file;
            std::string hsv_cluster = path;

            // 读取点云文件
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

            // 定义边界框最小和最大坐标
            pcl::PointXYZRGB minPt, maxPt;
            pcl::getMinMax3D(*cloud_bbox, minPt, maxPt);

            // 定义边界框的最小和最大坐标
            float min_x = minPt.x - 0.04;
            float min_y = minPt.y - 0.04;
            float min_z = minPt.z - 0.2;
            float max_x = maxPt.x + 0.04;
            float max_y = maxPt.y + 0.04;
            float max_z = maxPt.z + 0.2;

            // 创建新的点云对象，用于存放分割出的点
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // 遍历原始点云，将在边界框内的点加入新的点云中
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

            //// 设置滤波器参数
            s.setMeanK(50); // 邻域点的数量默认50
            s.setStddevMulThresh(4); // 标准差倍数阈值3

            // 执行滤波操作
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
            s.filter(*cloud_filtered_seg);

            // 设置新点云的宽度和高度
            //segmented_cloud->width = segmented_cloud->points.size();
            //segmented_cloud->height = 1;

            // 打印分割后点云的信息
            std::cout << "Filtered segmented cloud points count: " << cloud_filtered_seg->points.size() << std::endl;
            std::ostringstream oss1;

            oss1 << usfile << i << ".ply";
            std::string final_cluster = oss1.str();
            // 保存分割后的点云
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
