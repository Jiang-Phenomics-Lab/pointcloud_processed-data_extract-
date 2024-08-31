#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

int main() {
    // 硬编码的输入输出文件路径
    std::string input_file = "7pn_hsv_ro.pcd";  // 替换为你的输入文件路径
    std::string output_file = "7pn_tran.pcd"; // 替换为你想要保存的输出文件路径

    // 定义点云类型
    typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

    // 创建点云对象
    Cloud::Ptr cloud(new Cloud());

    // 从文件读取点云
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", input_file.c_str());
        return (-1);
    }

    // 定义变换矩阵
    Eigen::Matrix4f transformation_matrix;
    transformation_matrix << 0.984807729721, -0.086824089289, 0.150383725762, 0.167047739029,
        0.000000000000, 0.866025388241, 0.500000000000, 0.404293179512,
        -0.173648178577, -0.492403864861, 0.852868556976, 0.400521695614,
        0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;

    // 应用变换矩阵到点云
    Cloud::Ptr cloud_transformed(new Cloud());
    pcl::transformPointCloud(*cloud, *cloud_transformed, transformation_matrix);

    // 保存变换后的点云到新文件
    if (pcl::io::savePCDFile(output_file, *cloud_transformed) == -1) {
        PCL_ERROR("Couldn't write file %s \n", output_file.c_str());
        return (-1);
    }

    // 输出完成
    std::cout << "Finished transforming and saving point cloud data." << std::endl;

    return 0;
}