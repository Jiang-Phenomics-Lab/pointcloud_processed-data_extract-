#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

int main()
{
    for (int i = 7; i < 12; i++) {
        // Ӳ��������ļ�·��
        std::ostringstream oss;
        oss << "./final_dataset/0520_liaodou50_ds_sf_hsv_" << i << ".ply";
        std::string path = oss.str();
        std::string filename = "./final_dataset/0520_liaodou50.ply";
        std::string filename1 = path;
        // ��ȡ�����ļ�
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
        {
            std::cerr << "Error reading PLY file " << filename << std::endl;
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1)
        {
            std::cerr << "Error reading PLY file " << filename1 << std::endl;
            return -1;
        }

        // ����߽����С���������
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud1, minPt, maxPt);

        // ����߽�����С���������
        float min_x = minPt.x;
        float min_y = minPt.y;
        float min_z = minPt.z;
        float max_x = maxPt.x;
        float max_y = maxPt.y;
        float max_z = maxPt.z;

        // �����µĵ��ƶ������ڴ�ŷָ���ĵ�
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // ����ԭʼ���ƣ����ڱ߽���ڵĵ�����µĵ�����
        for (const pcl::PointXYZRGB& point : cloud->points)
        {
            if (point.x >= min_x && point.x <= max_x &&
                point.y >= min_y && point.y <= max_y &&
                point.z >= min_z && point.z <= max_z)
            {
                segmented_cloud->points.push_back(point);
            }
        }

        // �����µ��ƵĿ�Ⱥ͸߶�
        segmented_cloud->width = segmented_cloud->points.size();
        segmented_cloud->height = 1;

        // ��ӡ�ָ����Ƶ���Ϣ
        std::cout << "�ָ������еĵ�����" << segmented_cloud->points.size() << std::endl;
        std::ostringstream oss1;
        oss1 << "./final_dataset/0520_liaodou50_us_" << i << ".ply";
        std::string path1 = oss1.str();
        // ����ָ��ĵ���
        pcl::io::savePLYFile(path1, *segmented_cloud);
        
    }
    return 0;
}
