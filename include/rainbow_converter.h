#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

class RainbowConverter {
public:
    RainbowConverter(float min_distance = 0.5f, float max_distance = 5.0f);
    
    // Convert single PCD file to rainbow depth image
    bool convertPcdToRainbow(const std::string& pcd_path, const std::string& output_path);
    
    // Convert all PCD files in a directory
    void convertDirectory(const std::string& input_dir, const std::string& output_dir);
    
    // Set distance range for color mapping
    void setDistanceRange(float min_dist, float max_dist);

private:
    float min_color_distance_;
    float max_color_distance_;
    
    // Create rainbow colormap lookup table
    cv::Mat createRainbowLUT();
    
    // Convert point cloud to depth image
    cv::Mat pointCloudToDepthImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   int width, int height);
    
    // Apply rainbow colormap to depth image
    cv::Mat applyRainbowColormap(const cv::Mat& depth_image);
    
    // Get all PCD files in directory
    std::vector<std::string> getPcdFiles(const std::string& directory);
};