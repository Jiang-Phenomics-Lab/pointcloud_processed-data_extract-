#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/distances.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;

// ��ȡ������z=0.3�ĵ�
void extractPointsWithZ(const Cloud::Ptr& cloud, Cloud::Ptr& extracted_cloud, float z_value1, float z_value2) {
    // ����ֱͨ�˲�������
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);

    // ���ù����ֶ�Ϊz�������ù�������Ϊz_value
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_value1, z_value2);

    // ִ�й���
    pass.filter(*extracted_cloud);
}

// �ҳ���������Զ���㲢�������
void findFarthestPoints(const Cloud::Ptr& cloud, PointType& point1, PointType& point2, double& max_distance) {
    max_distance = 0.0;
    if (cloud->size() < 2) return;

    // �����������������е�����ҵ���Զ����
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
    // ���ص���
    Cloud::Ptr cloud(new Cloud());
    pcl::io::loadPLYFile("0603_zhongdou41_6_label2_transformed_minz.ply", *cloud);

    // ��ȡz=0.3�ĵ�
    Cloud::Ptr extracted_cloud(new Cloud());
    extractPointsWithZ(cloud, extracted_cloud, 0.05f,0.052f);

    // �ҳ���Զ����
    PointType point1, point2;
    double max_distance;
    findFarthestPoints(extracted_cloud, point1, point2, max_distance);

    std::cout << "Max distance between two points: " << max_distance << std::endl;

    // ���ӻ����
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addPointCloud(cloud, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");

    // ������Զ����֮��ĺ���
    pcl::PointXYZRGB line_start = point1, line_end = point2;
    viewer.addLine(line_start, line_end, 1.0, 0.0, 0.0, "farthest_line");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return 0;
}