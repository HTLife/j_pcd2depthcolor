#include "rainbow_converter.h"
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <chrono>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace fs = boost::filesystem;

RainbowConverter::RainbowConverter(float min_distance, float max_distance)
    : min_color_distance_(min_distance), max_color_distance_(max_distance) {
}

void RainbowConverter::setDistanceRange(float min_dist, float max_dist) {
    min_color_distance_ = min_dist;
    max_color_distance_ = max_dist;
}

cv::Mat RainbowConverter::createRainbowLUT() {
    cv::Mat lut(256, 1, CV_8UC3);
    
    // Define rainbow colors (BGR format for OpenCV)
    std::vector<cv::Vec3b> colors = {
        cv::Vec3b(255, 0, 0),    // blue
        cv::Vec3b(255, 255, 0),  // cyan
        cv::Vec3b(0, 255, 255),  // yellow
        cv::Vec3b(0, 165, 255),  // orange
        cv::Vec3b(0, 0, 255),    // red
        cv::Vec3b(64, 64, 128)   // violet
    };
    
    // Create smooth interpolation between colors
    for (int i = 0; i < 256; i++) {
        float normalized = static_cast<float>(i) / 255.0f;
        float scaled = normalized * (colors.size() - 1);
        int lower_idx = static_cast<int>(std::floor(scaled));
        int upper_idx = std::min(lower_idx + 1, static_cast<int>(colors.size() - 1));
        float weight = scaled - lower_idx;
        
        if (lower_idx == upper_idx) {
            lut.at<cv::Vec3b>(i, 0) = colors[lower_idx];
        } else {
            cv::Vec3b lower_color = colors[lower_idx];
            cv::Vec3b upper_color = colors[upper_idx];
            cv::Vec3b interpolated;
            
            for (int c = 0; c < 3; c++) {
                interpolated[c] = static_cast<uchar>(
                    (1.0f - weight) * lower_color[c] + weight * upper_color[c]
                );
            }
            lut.at<cv::Vec3b>(i, 0) = interpolated;
        }
    }
    
    return lut;
}

cv::Mat RainbowConverter::pointCloudToDepthImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 int width, int height) {
    cv::Mat depth_image = cv::Mat::zeros(height, width, CV_32F);
    
    if (cloud->empty()) {
        return depth_image;
    }
    
    // Check if it's an organized point cloud
    if (cloud->isOrganized() && static_cast<int>(cloud->width) == width && 
        static_cast<int>(cloud->height) == height) {
        // Direct mapping for organized point clouds (fastest case)
        #pragma omp parallel for
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const auto& point = cloud->at(x, y);
                if (std::isfinite(point.z)) {
                    depth_image.at<float>(y, x) = point.z;
                }
            }
        }
    } else {
        // For unorganized point clouds, create projection
        // Find bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        
        float x_range = max_pt.x - min_pt.x;
        float y_range = max_pt.y - min_pt.y;
        
        if (x_range > 0 && y_range > 0) {
            #pragma omp parallel for
            for (size_t i = 0; i < cloud->size(); i++) {
                const auto& point = cloud->at(i);
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    // Map to image coordinates
                    int img_x = static_cast<int>((point.x - min_pt.x) / x_range * (width - 1));
                    int img_y = static_cast<int>((point.y - min_pt.y) / y_range * (height - 1));
                    
                    // Clamp to image bounds
                    img_x = std::max(0, std::min(img_x, width - 1));
                    img_y = std::max(0, std::min(img_y, height - 1));
                    
                    depth_image.at<float>(img_y, img_x) = point.z;
                }
            }
        }
    }
    
    return depth_image;
}

cv::Mat RainbowConverter::applyRainbowColormap(const cv::Mat& depth_image) {
    cv::Mat result = cv::Mat::zeros(depth_image.size(), CV_8UC3);
    cv::Mat lut = createRainbowLUT();
    
    // Create mask for valid pixels
    cv::Mat valid_mask;
    cv::compare(depth_image, 0, valid_mask, cv::CMP_GT);
    
    if (cv::countNonZero(valid_mask) == 0) {
        return result; // Return black image if no valid pixels
    }
    
    // Clamp depth values to specified range
    cv::Mat clamped_depth;
    cv::max(depth_image, min_color_distance_, clamped_depth);
    cv::min(clamped_depth, max_color_distance_, clamped_depth);
    
    // Normalize to 0-255 range
    cv::Mat normalized = (clamped_depth - min_color_distance_) / 
                        (max_color_distance_ - min_color_distance_) * 255.0f;
    
    cv::Mat normalized_uchar;
    normalized.convertTo(normalized_uchar, CV_8U);
    
    // Apply LUT
    cv::applyColorMap(normalized_uchar, result, lut);
    
    // Set invalid pixels to black
    result.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
    
    return result;
}

bool RainbowConverter::convertPcdToRainbow(const std::string& pcd_path, const std::string& output_path) {
    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "Error loading PCD file: " << pcd_path << std::endl;
        return false;
    }
    
    if (cloud->empty()) {
        std::cerr << "Warning: Empty point cloud in " << pcd_path << std::endl;
        return false;
    }
    
    // Remove NaN points for unorganized clouds
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    
    // Get cloud dimensions
    int width = cloud->isOrganized() ? static_cast<int>(cloud->width) : 640;
    int height = cloud->isOrganized() ? static_cast<int>(cloud->height) : 480;
    
    // Convert to depth image
    cv::Mat depth_image = pointCloudToDepthImage(cloud, width, height);
    
    // Apply rainbow colormap
    cv::Mat colored_image = applyRainbowColormap(depth_image);
    
    // Save image
    if (!cv::imwrite(output_path, colored_image)) {
        std::cerr << "Error saving image: " << output_path << std::endl;
        return false;
    }
    
    return true;
}

std::vector<std::string> RainbowConverter::getPcdFiles(const std::string& directory) {
    std::vector<std::string> pcd_files;
    
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        std::cerr << "Directory does not exist: " << directory << std::endl;
        return pcd_files;
    }
    
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().string());
        }
    }
    
    std::sort(pcd_files.begin(), pcd_files.end());
    return pcd_files;
}

void RainbowConverter::convertDirectory(const std::string& input_dir, const std::string& output_dir) {
    // Create output directory
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }
    
    // Get all PCD files
    std::vector<std::string> pcd_files = getPcdFiles(input_dir);
    
    if (pcd_files.empty()) {
        std::cout << "No PCD files found in " << input_dir << std::endl;
        return;
    }
    
    std::cout << "Found " << pcd_files.size() << " PCD files to process" << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Process files in parallel
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < pcd_files.size(); i++) {
        const std::string& pcd_file = pcd_files[i];
        fs::path input_path(pcd_file);
        fs::path output_path = fs::path(output_dir) / (input_path.stem().string() + "_rainbow_depth.png");
        
        bool success = convertPcdToRainbow(pcd_file, output_path.string());
        
        if (success) {
            std::cout << "Processed " << (i + 1) << "/" << pcd_files.size() 
                      << ": " << input_path.filename() << std::endl;
        } else {
            std::cerr << "Failed to process: " << input_path.filename() << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Conversion completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Average time per file: " << duration.count() / pcd_files.size() << " ms" << std::endl;
}