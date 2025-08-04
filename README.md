# PCD to Rainbow Depth Converter (C++)

High-performance C++ implementation for converting PCD point cloud files to rainbow-colored depth images using PCL and OpenCV libraries.

## Features

- **Ultra-fast processing** using PCL and OpenCV optimized libraries
- **Parallel processing** with OpenMP for multi-core acceleration
- **Organized point cloud support** for direct depth mapping
- **Configurable distance range** for color mapping
- **Batch processing** of entire directories
- **Memory efficient** with optimized algorithms

## Dependencies

- PCL (Point Cloud Library) >= 1.8
- OpenCV >= 4.0
- Boost >= 1.58
- FLANN (Fast Library for Approximate Nearest Neighbors)
- OpenMP (optional, for parallel processing)
- CMake >= 3.10

## Installation

### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev libboost-all-dev libflann-dev libeigen3-dev
```

### Build
```bash
cd j_pcd2depthcolor
mkdir build && cd build
cmake -Wno-dev ..
make -j$(nproc)
```

### System-wide Installation
```bash
sudo make install
```
This installs the `j_pcd2depthcolor` binary to `/usr/local/bin` for system-wide access.

## Usage

```bash
j_pcd2depthcolor <input_directory> <output_directory> [min_distance] [max_distance]
```

### Parameters
- `input_directory`: Directory containing PCD files
- `output_directory`: Directory to save rainbow depth images  
- `min_distance`: Minimum distance for color mapping (default: 0.5)
- `max_distance`: Maximum distance for color mapping (default: 5.0)

### Example
```bash
# Convert all PCD files with default distance range
j_pcd2depthcolor ../01-pcd-abashed-afternoon ../01-color-abashed-afternoon

# Convert with custom distance range
j_pcd2depthcolor ../01-pcd-abashed-afternoon ../01-color-abashed-afternoon 1.0 10.0
```

## Performance

Expected performance improvements over Python implementation:
- **10-50x faster** processing speed
- **Parallel processing** utilizes all CPU cores
- **Optimized memory usage** with native C++ algorithms
- **Direct PCL integration** eliminates parsing overhead

## Output

Generates PNG images with rainbow color mapping:
- **Blue**: Closest distances
- **Cyan → Yellow → Orange → Red**: Increasing distances  
- **Violet**: Furthest distances
- **Black**: Invalid/missing depth data

## Color Distance Range

The `min_color_distance` and `max_color_distance` parameters control the color mapping:
- Values below `min_distance` use the same color (blue)
- Values above `max_distance` use the same color (violet)
- Values between the range are mapped across the rainbow spectrum
