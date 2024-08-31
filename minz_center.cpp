#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv) {
    // 定义点云类型，包含RGB信息
    //for (int i = 0; i < 7; i++) {
        typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

        // 创建点云对象
        PointCloud::Ptr cloud(new PointCloud());
        std::ostringstream oss;
        //oss << "upsampled_7pn_0510_" << i << ".ply";
        oss << "0603_zhongdou41_6_label2_transformed.ply";
        std::string path = oss.str();
        // 使用PLY读取器读取带RGB信息的点云
        pcl::PLYReader reader;
        reader.read(path, *cloud); // 替换为您的PLY文件路径


        // 找到点云中Z轴坐标最小的点
        pcl::PointXYZRGB min_z_point;
        float min_z = std::numeric_limits<float>::max();
        for (const auto& point : cloud->points) {
            if (point.z < min_z) {
                min_z = point.z;
                min_z_point = point;
            }
        }

        // 创建一个变换矩阵，将最小Z轴点移动到原点
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        transformation_matrix(0, 3) = -min_z_point.x;
        transformation_matrix(1, 3) = -min_z_point.y;
        transformation_matrix(2, 3) = -min_z_point.z;

        // 应用变换矩阵到点云
        pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
        std::ostringstream oss1;
        //oss1 << "7pn_0510_" << i << "_minz.ply";
        oss1 << "0603_zhongdou41_6_label2_transformed_minz.ply";
        std::string path1 = oss1.str();
        // 使用PLY读取器读取带RGB信息的点云
        // 保存变换后的点云到新的PLY文件
        pcl::PLYWriter writer;
        writer.write(path1, *cloud, true); // true 表示使用二进制格式写入
    //}
    return 0;
}