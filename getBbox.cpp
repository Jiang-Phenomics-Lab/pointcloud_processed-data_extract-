#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // Ӳ��������ļ�·��
    //std::string filename = "0603_jidou19_1_withlabel_proj.ply";
    std::string filename = "E:\\seg_leave\\lccp_segment_1.ply";
    typedef pcl::PointXYZRGB PointType;

    // ��ȡ�����ļ�
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
    {
        std::cerr << "Error reading PLY file " << filename << std::endl;
        return -1;
    }

    // ����߽��
    //pcl::PointXYZRGB minPt, maxPt;
    //pcl::getMinMax3D(*cloud, minPt, maxPt);

    // �����Χ�е���С������
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // �����Χ�е�8���ǵ�
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

    // ����PCLVisualizer����
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // ���ư�Χ�еı�
    viewer.addLine(points[0], points[1], 1.0, 0.0, 0.0, "line1"); // x��������
    viewer.addLine(points[1], points[2], 1.0, 0.0, 0.0, "line2"); // y��������
    viewer.addLine(points[2], points[3], 1.0, 0.0, 0.0, "line3"); // x�Ḻ����
    viewer.addLine(points[3], points[0], 1.0, 0.0, 0.0, "line4"); // y�Ḻ����
    viewer.addLine(points[4], points[5], 1.0, 0.0, 0.0, "line5"); // x��������
    viewer.addLine(points[5], points[6], 1.0, 0.0, 0.0, "line6"); // y��������
    viewer.addLine(points[6], points[7], 1.0, 0.0, 0.0, "line7"); // x�Ḻ����
    viewer.addLine(points[7], points[4], 1.0, 0.0, 0.0, "line8"); // y�Ḻ����
    viewer.addLine(points[0], points[4], 1.0, 0.0, 0.0, "line9"); // z��������
    viewer.addLine(points[1], points[5], 1.0, 0.0, 0.0, "line10");
    viewer.addLine(points[2], points[6], 1.0, 0.0, 0.0, "line11");
    viewer.addLine(points[3], points[7], 1.0, 0.0, 0.0, "line12"); // z�Ḻ����
    

    // ��ӡ�߽������
    //std::cout << "�߽����С����: " << minPt.x << " " << minPt.y << " " << minPt.z << std::endl;
    //std::cout << "�߽���������: " << maxPt.x << " " << maxPt.y << " " << maxPt.z << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
