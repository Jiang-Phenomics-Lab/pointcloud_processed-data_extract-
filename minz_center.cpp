#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv) {
    // ����������ͣ�����RGB��Ϣ
    //for (int i = 0; i < 7; i++) {
        typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

        // �������ƶ���
        PointCloud::Ptr cloud(new PointCloud());
        std::ostringstream oss;
        //oss << "upsampled_7pn_0510_" << i << ".ply";
        oss << "0603_zhongdou41_6_label2_transformed.ply";
        std::string path = oss.str();
        // ʹ��PLY��ȡ����ȡ��RGB��Ϣ�ĵ���
        pcl::PLYReader reader;
        reader.read(path, *cloud); // �滻Ϊ����PLY�ļ�·��


        // �ҵ�������Z��������С�ĵ�
        pcl::PointXYZRGB min_z_point;
        float min_z = std::numeric_limits<float>::max();
        for (const auto& point : cloud->points) {
            if (point.z < min_z) {
                min_z = point.z;
                min_z_point = point;
            }
        }

        // ����һ���任���󣬽���СZ����ƶ���ԭ��
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        transformation_matrix(0, 3) = -min_z_point.x;
        transformation_matrix(1, 3) = -min_z_point.y;
        transformation_matrix(2, 3) = -min_z_point.z;

        // Ӧ�ñ任���󵽵���
        pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
        std::ostringstream oss1;
        //oss1 << "7pn_0510_" << i << "_minz.ply";
        oss1 << "0603_zhongdou41_6_label2_transformed_minz.ply";
        std::string path1 = oss1.str();
        // ʹ��PLY��ȡ����ȡ��RGB��Ϣ�ĵ���
        // ����任��ĵ��Ƶ��µ�PLY�ļ�
        pcl::PLYWriter writer;
        writer.write(path1, *cloud, true); // true ��ʾʹ�ö����Ƹ�ʽд��
    //}
    return 0;
}