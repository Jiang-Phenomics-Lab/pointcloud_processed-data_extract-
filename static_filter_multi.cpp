#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

int main() {
    for (int i = 0; i < 6; i++) {
        // 读取点云数据
        std::ostringstream oss;
        oss << "./0529/0529_jidou19_us_" << i << "_rs.ply";
        std::string path = oss.str();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::io::loadPCDFile<pcl::PointXYZRGB>("0426.ply", *cloud);
        pcl::io::loadPLYFile<pcl::PointXYZRGB>(path, *cloud);
        // 创建统计滤波器对象
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);

        // 设置滤波器参数
        sor.setMeanK(0.6); // 邻域点的数量默认50
        sor.setStddevMulThresh(1); // 标准差倍数阈值

        // 执行滤波操作
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*cloud_filtered);
        std::ostringstream oss1;
        oss1 << "./0529/0529_jidou19_sf_" << i << ".ply";
        std::string path1 = oss1.str();
        // 输出滤波后的点云数据
        pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(path1, *cloud_filtered);

        std::cout << "PointCloud after filtering: " << cloud_filtered->size() << " data points." << std::endl;
    }
    return 0;
}
