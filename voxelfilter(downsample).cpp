#include <iostream>
#include <pcl/io/ply_io.h>  // ����PLY�ļ��Ķ�д
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv) {
    // ����������ͣ�����ʹ��PointXYZRGB
    typedef pcl::PointXYZRGB PointType;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());

    // PLY��ȡ��
    pcl::PLYReader reader;

    // ��ȡPLY�ļ�
    reader.read("./final_dataset/0520_kedou35.ply", *cloud); // ȷ���ļ�·����ȷ

    std::cerr << "PointCloud before filtering: " << cloud->size()
        << " data points." << std::endl;

    // �����˲���
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    // �������ش�С
    sor.setLeafSize(0.008f, 0.008f, 0.008f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->size()
        << " data points." << std::endl;

    // PLYд����
    pcl::PLYWriter writer;

    // д��PLY�ļ�
    writer.write("./final_dataset/0520_kedou35_ds.ply", *cloud_filtered, true); // true ��ʾʹ�ö����Ƹ�ʽд��

    return (0);
}