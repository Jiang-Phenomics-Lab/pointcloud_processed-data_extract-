#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointType; // 定义点云类型
typedef pcl::PointCloud<PointType> Cloud;

int main() {
    // 加载点云
    Cloud::Ptr cloud(new Cloud());
    pcl::io::loadPLYFile("C:\\Users\\74773\\Desktop\\0822_kedou35_2.ply", *cloud);

    // 创建一个投影矩阵，将Z坐标设置为0
    Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
    projection_matrix(2, 0) = 0; // X不变
    projection_matrix(2, 1) = 0; // Y不变
    projection_matrix(2, 2) = 0; // Z设为0
    projection_matrix(2, 3) = 0; // 保持原点

    // 创建一个新的点云对象来存储投影后的点云
    Cloud::Ptr projected_cloud(new Cloud());

    // 应用投影矩阵
    pcl::transformPointCloud(*cloud, *projected_cloud, projection_matrix);

    // 保存投影后的点云为新的文件
    pcl::io::savePLYFile("C:\\Users\\74773\\Desktop\\0822_kedou35_2.ply", *projected_cloud);

    return 0;
}