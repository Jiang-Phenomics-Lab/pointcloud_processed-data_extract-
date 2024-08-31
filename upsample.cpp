#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

int main(int argc, char** argv) {
    // Ӳ�������������ļ�·��
    std::string input_file = "7pn_0.pcd";  // �滻Ϊ�����������ļ�·��
    std::string output_file = "7pn_0(upsam).pcd"; // �滻Ϊ����Ҫ�������������ļ�·��

    // �����������
    typedef pcl::PointXYZRGB PointType;

    // �������ƶ���
    pcl::PointCloud<PointType> cloud;
    pcl::PointCloud<PointType> cloud_filtered;

    // ���ļ���ȡ����
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
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointType>::SAMPLE_LOCAL_PLANE); // �Ե��ƽ����ϲ���
    mls.setUpsamplingRadius(0.02);    // r2
    mls.setUpsamplingStepSize(0.0001);  // r3

    // ִ���ϲ�������
    mls.process(cloud_filtered);

    // �����ϲ�����ĵ��Ƶ����ļ�
    if (pcl::io::savePCDFile(output_file, cloud_filtered) == -1) {
        PCL_ERROR("Couldn't write file %s \n", output_file.c_str());
        return (-1);
    }

    // ������
    std::cout << "Finished upsampling point cloud data." << std::endl;

    return 0;
}