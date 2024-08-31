#include <vtkAutoInit.h>         
#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>		  
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

using namespace std;
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

void drawBoundingBox(pcl::visualization::PCLVisualizer &viewer, const pcl::PointCloud<PointType>::Ptr &cloud)
{
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
}

int main(int argc, char** argv)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr cloud_normal(new pcl::PointCloud<NormalType>());

	//std::string fileName = "0603_zhongdou41_6_label2.ply";
	std::string fileName = "E:\\seg_leave\\lccp_segment_1.ply";
	pcl::io::loadPLYFile(fileName, *cloud);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	//eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3, 3>(0, 0)) * (pcaCentroid.head<3>());// 

	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, transform);

	std::cout << eigenValuesPCA << std::endl;
	std::cout << eigenVectorsPCA << std::endl;

	//转换到原点时的主方向
	PointType o;
	o.x = 0.0;
	o.y = 0.0;
	o.z = 0.0;
	Eigen::Affine3f tra_aff(transform);
	Eigen::Vector3f pz = eigenVectorsPCA.col(0);
	Eigen::Vector3f py = eigenVectorsPCA.col(1);
	Eigen::Vector3f px = eigenVectorsPCA.col(2);
	pcl::transformVector(pz, pz, tra_aff);
	pcl::transformVector(py, py, tra_aff);
	pcl::transformVector(px, px, tra_aff);
	PointType pcaZ;
	pcaZ.x = 1000 * pz(0);
	pcaZ.y = 1000 * pz(1);
	pcaZ.z = 1000 * pz(2);
	PointType pcaY;
	pcaY.x = 1000 * py(0);
	pcaY.y = 1000 * py(1);
	pcaY.z = 1000 * py(2);
	PointType pcaX;
	pcaX.x = 1000 * px(0);
	pcaX.y = 1000 * px(1);
	pcaX.z = 1000 * px(2);

	//初始位置时的主方向
	PointType c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	PointType pcZ;
	pcZ.x = 1000 * eigenVectorsPCA(0, 0) + c.x;
	pcZ.y = 1000 * eigenVectorsPCA(1, 0) + c.y;
	pcZ.z = 1000 * eigenVectorsPCA(2, 0) + c.z;
	PointType pcY;
	pcY.x = 1000 * eigenVectorsPCA(0, 1) + c.x;
	pcY.y = 1000 * eigenVectorsPCA(1, 1) + c.y;
	pcY.z = 1000 * eigenVectorsPCA(2, 1) + c.z;
	PointType pcX;
	pcX.x = 1000 * eigenVectorsPCA(0, 2) + c.x;
	pcX.y = 1000 * eigenVectorsPCA(1, 2) + c.y;
	pcX.z = 1000 * eigenVectorsPCA(2, 2) + c.z;
	//visualization
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	float width = maxPt[0] - minPt[0];
	float height = maxPt[1] - minPt[1];
	float depth = maxPt[2] - minPt[2];
	std::cout << "Width: " << width << ", Height: " << height << ", Depth: " << depth << std::endl;


	// 创建PCLVisualizer对象
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	// 画出包围盒
	drawBoundingBox(viewer, cloud);

	// 设置背景颜色为白色
	viewer.setBackgroundColor(1.0, 1.0, 1.0);

	// 启动事件循环
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	// 保存变换后的点云
	//pcl::io::savePLYFileBinary("0603_zhongdou41_6_label2_transformed.ply", *transformedCloud);
	pcl::io::savePLYFileBinary("E:\\seg_leave\\lccp_segment_1_transformed.ply", *transformedCloud);

	return 0;
}