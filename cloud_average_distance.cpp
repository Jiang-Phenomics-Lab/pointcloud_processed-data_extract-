#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>


double ComputeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	//step1: �½�kdtree��������
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);

	//step2: ��������ÿ���㣬���ҳ�������������ĵ�
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		// ȡ�ڶ������룬��Ϊ��һ����������
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		//step3: ͳ����С����͡���Ч������
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}

	//step4: ����ռ�ֱ���
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
int main() {
    // ���������ļ�·��
    std::string inputFilePath = "cluster_0.pcd";

    // ���ص���
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(inputFilePath, *cloud) != 0) {
        PCL_ERROR("Error loading cloud file!\n");
        return -1;
    }

    // ������Ʒֱ���
    float resolution = ComputeCloudResolution(cloud);


    // ���ý�С֧�ְ뾶��Rs��Ϊ�ֱ��ʵ�һ��
    float Rs = resolution / 2.0;

    // ���ýϴ�֧�ְ뾶��Rl��Ϊ�ֱ��ʵ�����
    float Rl = resolution * 2.0;

    // �����С�ͽϴ�֧�ְ뾶
    std::cout << "Smaller support radius (Rs): " << Rs << std::endl;
    std::cout << "Larger support radius (Rl): " << Rl << std::endl;

    return 0;
}
