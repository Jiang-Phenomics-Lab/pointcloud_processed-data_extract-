#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/visualization/pcl_visualizer.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	pcl::PLYReader reader;
	// ��·����Ϊ�Լ�����ļ���·��
	reader.read<pcl::PointXYZ>("./final_dataset/0520_kedou46_us_1.ply", *cloud);
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	// �����˲�������
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);//���ô��˲��ĵ���
	sor.setMeanK(50);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ھӵ���
	sor.setStddevMulThresh(1.0);//�����ж��Ƿ�Ϊ��Ⱥ�����ֵ
	sor.filter(*cloud_filtered);//���˲����������cloud_filtered��
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("./final_dataset/0520_kedou46_us_1_1.ply", *cloud_filtered, false);
	sor.setNegative(true);


	pcl::visualization::PCLVisualizer viewer("statistic removal");
	int v1(1);
	int v2(2);
	//�����Ӵ�
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);

	viewer.addPointCloud(cloud, "cloud", v1);
	viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);

	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("./final_dataset/0520_kedou46_us_1_1_1.ply", *cloud_filtered, false);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}