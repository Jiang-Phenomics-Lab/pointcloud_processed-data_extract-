#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

namespace fs = boost::filesystem;

// �������ļ��ĺ���
void processFile(const fs::path& file_path) {
    try {
        std::cout << "�����ļ�: " << file_path << std::endl;

        // �������ƶ��������ָ��
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PLYReader reader;

        if (reader.read(file_path.string(), *cloud) == -1) {
            std::cerr << "�޷���ȡ�ļ� " << file_path << std::endl;
            return;
        }

        std::cout << "�������ݵ���: " << cloud->size() << std::endl;

        // ����ͳ���˲�������
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(100); // ����������
        sor.setStddevMulThresh(1.0); // ��׼�����ֵ

        // ִ���˲�����
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*cloud_filtered);

        std::cout << "�˲���������ݵ���: " << cloud_filtered->size() << std::endl;

        // �����µ��ļ�·��
        fs::path new_file_path = file_path.parent_path() / (file_path.stem().string() + "_sf" + file_path.extension().string());

        pcl::PLYWriter writer;
        if (writer.write(new_file_path.string(), *cloud_filtered) == -1) {
            std::cerr << "�޷�д���ļ� " << new_file_path.string() << std::endl;
            return;
        }

        std::cout << "�ļ��ѱ���: " << new_file_path.string() << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "�����ļ�ʱ��������: " << e.what() << std::endl;
    }
}

// ������
int main() {
    fs::path dir_path = ".\\0520_final"; // �滻Ϊ���Ŀ¼·��

    // ���Ŀ¼·���Ƿ���Ч
    if (!fs::exists(dir_path) || !fs::is_directory(dir_path)) {
        std::cerr << "Ŀ¼·����Ч��" << std::endl;
        return -1;
    }

    // ����Ŀ¼�е������ļ�
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".ply") {
            processFile(entry.path());
        }
    }

    std::cout << "�����ļ��������" << std::endl;
    return 0;
}
