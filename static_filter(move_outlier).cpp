#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

int main() {
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
    pcl::PLYReader reader;
    reader.read(".\\0703_WL82\\0703_WL82_6.ply", *cloud);
    //if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1.ply", *cloud) == -1) {
    //    // 创建统计滤波器对象
    //    PCL_ERROR("Couldn't read file input.pcd \n");
    //    return (-1);
    //}
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);

    // 设置滤波器参数
    sor.setMeanK(20); // 邻域点的数量默认50
    sor.setStddevMulThresh(5); // 标准差倍数阈值
    
    // 执行滤波操作
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);

    pcl::PLYWriter writer;
    writer.write(".\\0703_WL82\\0703_WL82_6_f.ply", *cloud_filtered);

    // 输出滤波后的点云数据
    //pcl::io::savePLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_1_sf.ply", *cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->size() << " data points." << std::endl;

    return 0;
}
