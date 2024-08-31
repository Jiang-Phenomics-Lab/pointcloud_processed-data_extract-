#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/search/kdtree.h>
#include <map>
#include <random>

typedef pcl::PointXYZRGB PointT;
typedef pcl::LCCPSegmentation<PointT> LCCPSegmentationT;
typedef LCCPSegmentationT::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

// 生成随机颜色
std::vector<uint8_t> getRandomColor() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255);
    return { static_cast<uint8_t>(dis(gen)), static_cast<uint8_t>(dis(gen)), static_cast<uint8_t>(dis(gen)) };
}

void colorCloudByLabels(const pcl::PointCloud<pcl::PointXYZL>& cloud, pcl::PointCloud<PointT>& colored_cloud) {
    std::map<uint32_t, std::vector<uint8_t>> label_to_color;
    for (const auto& point : cloud.points) {
        uint32_t label = point.label;
        if (label_to_color.find(label) == label_to_color.end()) {
            label_to_color[label] = getRandomColor();
        }
        PointT colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = label_to_color[label][0];
        colored_point.g = label_to_color[label][1];
        colored_point.b = label_to_color[label][2];
        colored_cloud.points.push_back(colored_point);
    }
}

void savePLY(const pcl::PointCloud<PointT>& cloud, const std::string& filename) {
    pcl::io::savePLYFile(filename, cloud);
}

int main() {
    // 定义输入输出文件名
    const std::string input_filename = "E:\\seg_leave\\leave.ply";
    const std::string output_filename_base = "E:\\seg_leave\\lccp_segment_";
    const std::string supervoxel_output_filename = "E:\\seg_leave\\super_voxel_leave.ply";

    // 加载输入点云
    pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPLYFile(input_filename, *input_cloud_ptr) < 0) {
        PCL_ERROR("Couldn't read input file");
        return (-1);
    }

    // 超体聚类参数设置
    float voxel_resolution = 0.0008f;
    float seed_resolution = 0.005f;
    float color_importance = 5.0f;
    float spatial_importance = 5.0f;
    float normal_importance = 5.0f;
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setInputCloud(input_cloud_ptr);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto& supervoxel : supervoxel_clusters) {
        uint32_t label = supervoxel.first;
        pcl::PointCloud<PointT>::Ptr cloud = supervoxel.second->voxels_;
        for (const auto& point : cloud->points) {
            pcl::PointXYZL point_l;
            point_l.x = point.x;
            point_l.y = point.y;
            point_l.z = point.z;
            point_l.label = label;
            supervoxel_labeled_cloud->points.push_back(point_l);
        }
    }
    pcl::io::savePLYFile(supervoxel_output_filename, *supervoxel_labeled_cloud);

    // 获取超体邻接信息
    SuperVoxelAdjacencyList sv_adjacency_list;
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // LCCP分割参数设置
    float concavity_tolerance_threshold = 0.05f;
    float smoothness_threshold = 0.005f;
    uint32_t min_segment_size = 1000;
    LCCPSegmentationT lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();

    // 从分割结果中获取标签点云
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    lccp.getSVAdjacencyList(sv_adjacency_list);
    const std::string output_all = "E:\\seg_leave\\leave_lccp.ply";
    pcl::io::savePLYFile(output_all, *lccp_labeled_cloud);

    // 创建kd-tree用于最近邻搜索
    pcl::search::KdTree<pcl::PointXYZL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZL>);
    tree->setInputCloud(lccp_labeled_cloud);

    float distance_threshold = 0.2; // 设定一个合理的距离阈值
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> label_to_cloud_map;

    // 清空点数小于500的类别的标签
    for (uint32_t label = 0; label <= supervoxel_clusters.size(); ++label) {
        pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        for (size_t i = 0; i < lccp_labeled_cloud->points.size(); ++i) {
            if (lccp_labeled_cloud->points[i].label == label) {
                segmented_cloud->points.push_back((*lccp_labeled_cloud)[i]);
            }
        }

        if (segmented_cloud->points.size() >= 5000) {
            label_to_cloud_map[label] = segmented_cloud;
            std::string output_filename = output_filename_base + std::to_string(label) + ".ply";
            pcl::io::savePLYFile(output_filename, *segmented_cloud);
        }
        else {
            for (size_t i = 0; i < segmented_cloud->points.size(); ++i) {
                if (segmented_cloud->points[i].label == label) {
                   
                    std::vector<int> nearest_indices(1);
                    std::vector<float> nearest_distances(1);

                    // 查找最近邻点
                    tree->nearestKSearch(segmented_cloud->points[i], 1, nearest_indices, nearest_distances);

                    // 获取最近邻点的标签
                    uint32_t nearest_label = lccp_labeled_cloud->points[nearest_indices[0]].label;

                    // 只更新为点云数量大于 5000 的类的标签

                    segmented_cloud->points[i].label = nearest_label;
                    cout << "change to" << nearest_label << endl;


                }
            }
            // 如果 `label` 已经存在于 `label_to_cloud_map` 中，则合并点云
            if (label_to_cloud_map.find(label) != label_to_cloud_map.end()) {
                *label_to_cloud_map[label] += *segmented_cloud;
                cout << "+=" << endl;
            }
            else {
                // 如果 `label` 不存在，则直接加入
                label_to_cloud_map[label] = segmented_cloud;
                cout << "new" << endl;
            }
            std::string output_filename = output_filename_base + std::to_string(label) + "_change.ply";
            pcl::io::savePLYFile(output_filename, *segmented_cloud);
        }
    }



    // 合并所有 segmented_cloud 成一个点云
    pcl::PointCloud<pcl::PointXYZL>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto& kv : label_to_cloud_map) {
        *merged_cloud += *kv.second;
    }

    // 保存最终的分割结果
    const std::string updated_output = "E:\\seg_leave\\leave_lccp_updated.ply";
    pcl::io::savePLYFile(updated_output, *merged_cloud);


    // 创建两个PCL Viewer并显示结果
    pcl::visualization::PCLVisualizer::Ptr viewer_supervoxel(new pcl::visualization::PCLVisualizer("Supervoxel Viewer"));
    viewer_supervoxel->setBackgroundColor(0, 0, 0);

    pcl::visualization::PCLVisualizer::Ptr viewer_lccp(new pcl::visualization::PCLVisualizer("LCCP Viewer"));
    viewer_lccp->setBackgroundColor(0, 0, 0);

    // 显示超体聚类结果
    pcl::PointCloud<PointT>::Ptr supervoxel_colored_cloud(new pcl::PointCloud<PointT>);
    colorCloudByLabels(*supervoxel_labeled_cloud, *supervoxel_colored_cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> supervoxel_color_handler(supervoxel_colored_cloud);
    viewer_supervoxel->addPointCloud<PointT>(supervoxel_colored_cloud, supervoxel_color_handler, "supervoxel_cloud");
    viewer_supervoxel->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "supervoxel_cloud");

    // 显示LCCP分割结果
    pcl::PointCloud<PointT>::Ptr lccp_colored_cloud(new pcl::PointCloud<PointT>);
    colorCloudByLabels(*lccp_labeled_cloud, *lccp_colored_cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> lccp_color_handler(lccp_colored_cloud);
    viewer_lccp->addPointCloud<PointT>(lccp_colored_cloud, lccp_color_handler, "lccp_cloud");
    viewer_lccp->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lccp_cloud");

    viewer_supervoxel->addCoordinateSystem(1.0);
    viewer_supervoxel->initCameraParameters();

    viewer_lccp->addCoordinateSystem(1.0);
    viewer_lccp->initCameraParameters();

    // 循环显示两个窗口
    while (!viewer_supervoxel->wasStopped() && !viewer_lccp->wasStopped()) {
        viewer_supervoxel->spinOnce(100);
        viewer_lccp->spinOnce(100);
    }

    return 0;
}
