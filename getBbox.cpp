#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // 硬编码点云文件路径
    //std::string filename = "0603_jidou19_1_withlabel_proj.ply";
    std::string filename = "E:\\seg_leave\\lccp_segment_1.ply";
    typedef pcl::PointXYZRGB PointType;

    // 读取点云文件
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
    {
        std::cerr << "Error reading PLY file " << filename << std::endl;
        return -1;
    }

    // 计算边界框
    //pcl::PointXYZRGB minPt, maxPt;
    //pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 计算包围盒的最小和最大点
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 定义包围盒的8个角点
    PointType points[8];
    points[0] = PointType(minPt[0], minPt[1], minPt[2]);
    points[1] = PointType(maxPt[0], minPt[1], minPt[2]);
    points[2] = PointType(maxPt[0], maxPt[1], minPt[2]);
    points[3] = PointType(minPt[0], maxPt[1], minPt[2]);
    points[4] = PointType(minPt[0], minPt[1], maxPt[2]);
    points[5] = PointType(maxPt[0], minPt[1], maxPt[2]);
    points[6] = PointType(maxPt[0], maxPt[1], maxPt[2]);
    points[7] = PointType(minPt[0], maxPt[1], maxPt[2]);


    float lenth = maxPt[0] - minPt[0];
    float width = maxPt[1] - minPt[1];
    float height = maxPt[2] - minPt[2];
    std::cout << "Lenth: " << lenth << ", Width: " << width << ", Height: " << height << std::endl;

    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // 绘制包围盒的边
    viewer.addLine(points[0], points[1], 1.0, 0.0, 0.0, "line1"); // x轴正方向
    viewer.addLine(points[1], points[2], 1.0, 0.0, 0.0, "line2"); // y轴正方向
    viewer.addLine(points[2], points[3], 1.0, 0.0, 0.0, "line3"); // x轴负方向
    viewer.addLine(points[3], points[0], 1.0, 0.0, 0.0, "line4"); // y轴负方向
    viewer.addLine(points[4], points[5], 1.0, 0.0, 0.0, "line5"); // x轴正方向
    viewer.addLine(points[5], points[6], 1.0, 0.0, 0.0, "line6"); // y轴正方向
    viewer.addLine(points[6], points[7], 1.0, 0.0, 0.0, "line7"); // x轴负方向
    viewer.addLine(points[7], points[4], 1.0, 0.0, 0.0, "line8"); // y轴负方向
    viewer.addLine(points[0], points[4], 1.0, 0.0, 0.0, "line9"); // z轴正方向
    viewer.addLine(points[1], points[5], 1.0, 0.0, 0.0, "line10");
    viewer.addLine(points[2], points[6], 1.0, 0.0, 0.0, "line11");
    viewer.addLine(points[3], points[7], 1.0, 0.0, 0.0, "line12"); // z轴负方向
    

    // 打印边界框坐标
    //std::cout << "边界框最小坐标: " << minPt.x << " " << minPt.y << " " << minPt.z << std::endl;
    //std::cout << "边界框最大坐标: " << maxPt.x << " " << maxPt.y << " " << maxPt.z << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
