#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

using namespace std;
using namespace pcl;

// 计算点的EXG值
double calculate_EXG(const PointXYZRGB& point) {
    double R = point.r / 255.0;
    double G = point.g / 255.0;
    double B = point.b / 255.0;
    return 2 * G - R - B;
}

int main() {
    // 加载点云数据
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    if (io::loadPCDFile<PointXYZRGB>("p21_ds_rs_hsv.pcd", *cloud) == -1) {
        cerr << "Failed to read point cloud file." << endl;
        return -1;
    }

    // 计算每个点的EXG值
    vector<double> exg_values;
    for (const auto& point : cloud->points) {
        double exg_value = calculate_EXG(point);
        exg_values.push_back(exg_value);
    }

    // 统计EXG值的频数，计算概率
    int L = 256; // EXG值的范围
    vector<int> exg_histogram(L, 0);
    int N = exg_values.size();
    for (int i = 0; i < N; ++i) {
        int exg_index = static_cast<int>(round(exg_values[i]));
        if (exg_index < 0) exg_index = 0;
        if (exg_index >= L) exg_index = L - 1;
        exg_histogram[exg_index]++;
    }
    vector<double> exg_probabilities(L, 0.0);
    for (int i = 0; i < L; ++i) {
        if (N != 0) // 避免除以零
            exg_probabilities[i] = static_cast<double>(exg_histogram[i]) / N;
    }

    // 计算类间方差，筛选最大方差对应的阈值
    double max_variance = -1;
    int optimal_threshold = 127; // 将阈值设为固定值
    double omega0 = 0, omega1 = 0;
    double mu0 = 0, mu1 = 0;
    for (int i = 0; i <= optimal_threshold; ++i) {
        omega0 += exg_probabilities[i];
        mu0 += i * exg_probabilities[i];
    }
    for (int i = optimal_threshold + 1; i < L; ++i) {
        omega1 += exg_probabilities[i];
        mu1 += i * exg_probabilities[i];
    }
    if (omega0 != 0 && omega1 != 0) { // 避免非法操作
        mu0 /= omega0;
        mu1 /= omega1;
        double variance = omega0 * omega1 * (mu0 - mu1) * (mu0 - mu1);
        if (!isnan(variance)) { // 检查结果是否为NaN
            max_variance = variance;
        }
    }

    cout << "Optimal threshold: " << optimal_threshold << endl;
    cout << "Class variance: " << max_variance << endl;

    // 通过最佳阈值T*提取植被点云
    PointCloud<PointXYZRGB>::Ptr vegetation_cloud(new PointCloud<PointXYZRGB>);
    for (const auto& point : cloud->points) {
        double exg_value = calculate_EXG(point);
        if (exg_value <= optimal_threshold) {
            vegetation_cloud->points.push_back(point);
        }
    }

    // 保存植被点云
    io::savePCDFileBinary("vegetation_cloud.pcd", *vegetation_cloud);

    return 0;
}
