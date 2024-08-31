#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

namespace fs = boost::filesystem;

// 处理单个文件的函数
void processFile(const fs::path& file_path) {
    try {
        std::cout << "处理文件: " << file_path << std::endl;

        // 创建点云对象的智能指针
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PLYReader reader;

        if (reader.read(file_path.string(), *cloud) == -1) {
            std::cerr << "无法读取文件 " << file_path << std::endl;
            return;
        }

        std::cout << "点云数据点数: " << cloud->size() << std::endl;

        // 创建统计滤波器对象
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(100); // 邻域点的数量
        sor.setStddevMulThresh(1.0); // 标准差倍数阈值

        // 执行滤波操作
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*cloud_filtered);

        std::cout << "滤波后点云数据点数: " << cloud_filtered->size() << std::endl;

        // 构造新的文件路径
        fs::path new_file_path = file_path.parent_path() / (file_path.stem().string() + "_sf" + file_path.extension().string());

        pcl::PLYWriter writer;
        if (writer.write(new_file_path.string(), *cloud_filtered) == -1) {
            std::cerr << "无法写入文件 " << new_file_path.string() << std::endl;
            return;
        }

        std::cout << "文件已保存: " << new_file_path.string() << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "处理文件时发生错误: " << e.what() << std::endl;
    }
}

// 主函数
int main() {
    fs::path dir_path = ".\\0520_final"; // 替换为你的目录路径

    // 检查目录路径是否有效
    if (!fs::exists(dir_path) || !fs::is_directory(dir_path)) {
        std::cerr << "目录路径无效！" << std::endl;
        return -1;
    }

    // 遍历目录中的所有文件
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".ply") {
            processFile(entry.path());
        }
    }

    std::cout << "所有文件处理完毕" << std::endl;
    return 0;
}
