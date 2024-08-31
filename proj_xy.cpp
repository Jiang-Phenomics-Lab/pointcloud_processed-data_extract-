#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointType; // �����������
typedef pcl::PointCloud<PointType> Cloud;

int main() {
    // ���ص���
    Cloud::Ptr cloud(new Cloud());
    pcl::io::loadPLYFile("C:\\Users\\74773\\Desktop\\0822_kedou35_2.ply", *cloud);

    // ����һ��ͶӰ���󣬽�Z��������Ϊ0
    Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
    projection_matrix(2, 0) = 0; // X����
    projection_matrix(2, 1) = 0; // Y����
    projection_matrix(2, 2) = 0; // Z��Ϊ0
    projection_matrix(2, 3) = 0; // ����ԭ��

    // ����һ���µĵ��ƶ������洢ͶӰ��ĵ���
    Cloud::Ptr projected_cloud(new Cloud());

    // Ӧ��ͶӰ����
    pcl::transformPointCloud(*cloud, *projected_cloud, projection_matrix);

    // ����ͶӰ��ĵ���Ϊ�µ��ļ�
    pcl::io::savePLYFile("C:\\Users\\74773\\Desktop\\0822_kedou35_2.ply", *projected_cloud);

    return 0;
}