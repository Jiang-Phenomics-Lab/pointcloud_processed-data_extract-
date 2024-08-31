#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

int main(int argc, char** argv) {
    // 硬编码的输入输出文件路径
    std::string input_file = "7pn_0.pcd";  // 替换为你的输入点云文件路径
    std::string output_file = "7pn_0(upsam).pcd"; // 替换为你想要保存的输出点云文件路径

    // 定义点云类型
    typedef pcl::PointXYZRGB PointType;

    // 创建点云对象
    pcl::PointCloud<PointType> cloud;
    pcl::PointCloud<PointType> cloud_filtered;

    // 从文件读取点云
    if (pcl::io::loadPCDFile<PointType>(input_file, cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", input_file.c_str());
        return (-1);
    }

    // Create a KD-Tree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, PointType> mls;
    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud.makeShared());
    mls.setPolynomialOrder(3);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.04); // r1

    // Upsampling
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointType>::SAMPLE_LOCAL_PLANE); // 对点云进行上采样
    mls.setUpsamplingRadius(0.02);    // r2
    mls.setUpsamplingStepSize(0.0001);  // r3

    // 执行上采样操作
    mls.process(cloud_filtered);

    // 保存上采样后的点云到新文件
    if (pcl::io::savePCDFile(output_file, cloud_filtered) == -1) {
        PCL_ERROR("Couldn't write file %s \n", output_file.c_str());
        return (-1);
    }

    // 输出完成
    std::cout << "Finished upsampling point cloud data." << std::endl;

    return 0;
}