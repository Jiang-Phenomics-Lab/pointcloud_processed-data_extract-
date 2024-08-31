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
	//step1: 新建kdtree用于搜索
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);

	//step2: 遍历点云每个点，并找出与它距离最近的点
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		// 取第二个距离，因为第一个是它本身
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		//step3: 统计最小距离和、有效点数量
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}

	//step4: 计算空间分辨率
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
int main() {
    // 定义输入文件路径
    std::string inputFilePath = "cluster_0.pcd";

    // 加载点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(inputFilePath, *cloud) != 0) {
        PCL_ERROR("Error loading cloud file!\n");
        return -1;
    }

    // 计算点云分辨率
    float resolution = ComputeCloudResolution(cloud);


    // 设置较小支持半径（Rs）为分辨率的一半
    float Rs = resolution / 2.0;

    // 设置较大支持半径（Rl）为分辨率的两倍
    float Rl = resolution * 2.0;

    // 输出较小和较大支持半径
    std::cout << "Smaller support radius (Rs): " << Rs << std::endl;
    std::cout << "Larger support radius (Rl): " << Rl << std::endl;

    return 0;
}
