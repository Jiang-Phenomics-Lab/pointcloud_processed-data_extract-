#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

int main()
{
    for (int i = 7; i < 12; i++) {
        // 硬编码点云文件路径
        std::ostringstream oss;
        oss << "./final_dataset/0520_liaodou50_ds_sf_hsv_" << i << ".ply";
        std::string path = oss.str();
        std::string filename = "./final_dataset/0520_liaodou50.ply";
        std::string filename1 = path;
        // 读取点云文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
        {
            std::cerr << "Error reading PLY file " << filename << std::endl;
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1)
        {
            std::cerr << "Error reading PLY file " << filename1 << std::endl;
            return -1;
        }

        // 定义边界框最小和最大坐标
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud1, minPt, maxPt);

        // 定义边界框的最小和最大坐标
        float min_x = minPt.x;
        float min_y = minPt.y;
        float min_z = minPt.z;
        float max_x = maxPt.x;
        float max_y = maxPt.y;
        float max_z = maxPt.z;

        // 创建新的点云对象，用于存放分割出的点
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 遍历原始点云，将在边界框内的点加入新的点云中
        for (const pcl::PointXYZRGB& point : cloud->points)
        {
            if (point.x >= min_x && point.x <= max_x &&
                point.y >= min_y && point.y <= max_y &&
                point.z >= min_z && point.z <= max_z)
            {
                segmented_cloud->points.push_back(point);
            }
        }

        // 设置新点云的宽度和高度
        segmented_cloud->width = segmented_cloud->points.size();
        segmented_cloud->height = 1;

        // 打印分割后点云的信息
        std::cout << "分割后点云中的点数：" << segmented_cloud->points.size() << std::endl;
        std::ostringstream oss1;
        oss1 << "./final_dataset/0520_liaodou50_us_" << i << ".ply";
        std::string path1 = oss1.str();
        // 保存分割后的点云
        pcl::io::savePLYFile(path1, *segmented_cloud);
        
    }
    return 0;
}
