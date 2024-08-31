#include <iostream>
#include <pcl/io/ply_io.h>  // 用于PLY文件的读写
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv) {
    // 定义点云类型，这里使用PointXYZRGB
    typedef pcl::PointXYZRGB PointType;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());

    // PLY读取器
    pcl::PLYReader reader;

    // 读取PLY文件
    reader.read("./final_dataset/0520_kedou35.ply", *cloud); // 确保文件路径正确

    std::cerr << "PointCloud before filtering: " << cloud->size()
        << " data points." << std::endl;

    // 体素滤波器
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    // 设置体素大小
    sor.setLeafSize(0.008f, 0.008f, 0.008f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->size()
        << " data points." << std::endl;

    // PLY写入器
    pcl::PLYWriter writer;

    // 写入PLY文件
    writer.write("./final_dataset/0520_kedou35_ds.ply", *cloud_filtered, true); // true 表示使用二进制格式写入

    return (0);
}