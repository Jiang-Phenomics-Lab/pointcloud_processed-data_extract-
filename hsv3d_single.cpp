#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <vector>

// Function to convert RGB to HSV
void rgbToHsv(float r, float g, float b, float& h, float& s, float& v) {
    r = r / 255.0;
    g = g / 255.0;
    b = b / 255.0;
    float max = std::max(r, std::max(g, b)), min = std::min(r, std::min(g, b));
    v = max;

    float diff = max - min;
    s = max == 0 ? 0 : diff / max;

    if (max == min) {
        h = 0; // achromatic
    }
    else {
        if (max == r) {
            h = (g - b) / diff + (g < b ? 6 : 0);
        }
        else if (max == g) {
            h = (b - r) / diff + 2;
        }
        else if (max == b) {
            h = (r - g) / diff + 4;
        }
        h /= 6;
    }
    h *= 360; // Convert to degrees
}

int main(int argc, char** argv) {
    // Load input point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_2.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return (-1);
    }

    // Filtered point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Filter points based on HSV values
    for (const auto& point : *cloud) {
        float h, s, v;
        rgbToHsv(point.r, point.g, point.b, h, s, v);

        // Check if the hue value is within the desired range
        if (h >= 50 && h <= 170 && s >= 0.25 && v <= 0.7 && v >= 0.1) {
            filtered_cloud->push_back(point);
        }
    }

    // Save the filtered point cloud
    pcl::io::savePLYFile<pcl::PointXYZRGB>("./final_dataset/0520_kedou46_us_2_hsv.ply", *filtered_cloud);

    std::cout << "Filtered point cloud saved as filtered_output.pcd" << std::endl;

    return 0;
}
