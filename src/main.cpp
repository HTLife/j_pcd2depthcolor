#include "rainbow_converter.h"
#include <iostream>
#include <string>

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <input_directory> <output_directory> [min_distance] [max_distance]" << std::endl;
    std::cout << "  input_directory: Directory containing PCD files" << std::endl;
    std::cout << "  output_directory: Directory to save rainbow depth images" << std::endl;
    std::cout << "  min_distance: Minimum distance for color mapping (default: 0.5)" << std::endl;
    std::cout << "  max_distance: Maximum distance for color mapping (default: 5.0)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << program_name << " ./pcd_files ./rainbow_images 0.5 5.0" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string input_dir = argv[1];
    std::string output_dir = argv[2];
    
    // Parse optional distance parameters
    float min_distance = 0.5f;
    float max_distance = 5.0f;
    
    if (argc >= 4) {
        try {
            min_distance = std::stof(argv[3]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid min_distance value: " << argv[3] << std::endl;
            return 1;
        }
    }
    
    if (argc >= 5) {
        try {
            max_distance = std::stof(argv[4]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid max_distance value: " << argv[4] << std::endl;
            return 1;
        }
    }
    
    if (min_distance >= max_distance) {
        std::cerr << "Error: min_distance must be less than max_distance" << std::endl;
        return 1;
    }
    
    std::cout << "PCD to Rainbow Depth Converter" << std::endl;
    std::cout << "Input directory: " << input_dir << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;
    std::cout << "Distance range: [" << min_distance << ", " << max_distance << "]" << std::endl;
    std::cout << std::endl;
    
    // Create converter and process files
    RainbowConverter converter(min_distance, max_distance);
    converter.convertDirectory(input_dir, output_dir);
    
    return 0;
}