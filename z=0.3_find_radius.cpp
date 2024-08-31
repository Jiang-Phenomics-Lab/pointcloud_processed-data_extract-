#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/distances.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;

// 截取点云中z=0.3的点
void extractPointsWithZ(const Cloud::Ptr& cloud, Cloud::Ptr& extracted_cloud, float z_value1, float z_value2) {
    // 创建直通滤波器对象
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);

    // 设置过滤字段为z，并设置过滤限制为z_value
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_value1, z_value2);

    // 执行过滤
    pass.filter(*extracted_cloud);
}

// 找出点云中最远两点并计算距离
void findFarthestPoints(const Cloud::Ptr& cloud, PointType& point1, PointType& point2, double& max_distance) {
    max_distance = 0.0;
    if (cloud->size() < 2) return;

    // 暴力方法：遍历所有点对以找到最远距离
    for (size_t i = 0; i < cloud->size(); ++i) {
        for (size_t j = i + 1; j < cloud->size(); ++j) {
            double distance = pcl::euclideanDistance((*cloud)[i], (*cloud)[j]);
            if (distance > max_distance) {
                point1 = (*cloud)[i];
                point2 = (*cloud)[j];
                max_distance = distance;
            }
        }
    }
}

int main() {
    // 加载点云
    Cloud::Ptr cloud(new Cloud());
    pcl::io::loadPLYFile("0603_zhongdou41_6_label2_transformed_minz.ply", *cloud);

    // 截取z=0.3的点
    Cloud::Ptr extracted_cloud(new Cloud());
    extractPointsWithZ(cloud, extracted_cloud, 0.05f,0.052f);

    // 找出最远两点
    PointType point1, point2;
    double max_distance;
    findFarthestPoints(extracted_cloud, point1, point2, max_distance);

    std::cout << "Max distance between two points: " << max_distance << std::endl;

    // 可视化结果
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addPointCloud(cloud, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");

    // 绘制最远两点之间的红线
    pcl::PointXYZRGB line_start = point1, line_end = point2;
    viewer.addLine(line_start, line_end, 1.0, 0.0, 0.0, "farthest_line");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return 0;
}