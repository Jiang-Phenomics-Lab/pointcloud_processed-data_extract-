#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // PointXYZRGB型点云

    pcl::PLYReader reader;
    reader.read("E:\\seg_leave\\0718_WL82_2 - Cloud - Cloud.ply", *cloud);


    // 体素网格滤波
    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud(cloud);
    //sor.setLeafSize(0.005f, 0.005f, 0.005f);
    //sor.filter(*cloud);


    // 创建一个空的kd-tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    // 创建一个normal点云
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator; // 创建法线估计对象
    normal_estimator.setSearchMethod(tree); // 设置搜索方法
    normal_estimator.setInputCloud(cloud); // 设置法线估计对象输入点集
    normal_estimator.setRadiusSearch(0.005); // 使用半径在查询点周围2厘米范围内的所有邻元素
    normal_estimator.compute(*cloud_normals); // 计算并输出法向量

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg; // 创造区域生长分割对象
    reg.setMinClusterSize(350); // 设置一个聚类需要的最小点数，聚类小于阈值的结果将被舍弃
    reg.setMaxClusterSize(100000); //设置一个聚类需要的最大点数，聚类大于阈值的结果将被舍弃
    reg.setSearchMethod(tree); // 设置搜索方法
    reg.setResidualThreshold(0.05); // 设置搜索的近邻点数目

    reg.setInputCloud(cloud); // 设置输入点云 
    reg.setInputNormals(cloud_normals); // 设置输入点云

    reg.setSmoothnessThreshold(3 / 180.0 * M_PI); //设置平滑阀值
    reg.setCurvatureThreshold(0.5); //设置曲率阀值


    // 以下两行用于启动分割算法，并返回聚类向量
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters); // 获取聚类的结果，分割结果保存在点云索引的向量中

    //区域生长结果可视化

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer3("Cloud Viewer"); // 创建一个可视化窗口
    viewer3.showCloud(colored_cloud);
    while (!viewer3.wasStopped())
    {
    }


    // 遍历每个聚类
    for (size_t i = 0; i < clusters.size(); ++i) {
        // 创建一个新的PointCloud对象
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 根据聚类索引填充新的PointCloud对象
        for (const auto& index : clusters[i].indices) {
            cluster_cloud->points.push_back(cloud->points[index]);
        }

        // 为每个聚类创建一个文件名
        std::string filename = "E:\\seg_leave\\0718_WL82_2 - Cloud - Cloud_" + std::to_string(i) + ".ply";

        // 使用PLYWriter保存聚类为文件
        pcl::PLYWriter writer;
        writer.write(filename, *cluster_cloud);
    }
}
