cmake_minimum_required(VERSION 3.10)

# Set policy for FLANN compatibility
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(j_pcd2depthcolor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
set(CMAKE_BUILD_TYPE Release)

# Find required packages
find_package(PCL 1.8 QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)
find_package(Boost REQUIRED COMPONENTS system filesystem)

if(NOT PCL_FOUND)
    message(WARNING "PCL not found. Install with: sudo apt-get install libpcl-dev")
    message(FATAL_ERROR "PCL is required for this project")
endif()

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_CFLAGS_OTHER})

# Add executable
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/rainbow_converter.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Enable parallel processing
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# Install binary to system path
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# CPack configuration for .deb package
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "j-pcd2depthcolor")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_CONTACT "maintainer@example.com")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PCD to depth color converter")
set(CPACK_PACKAGE_DESCRIPTION "A tool to convert PCD files to depth color images")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpcl-dev, libopencv-dev, libboost-system-dev, libboost-filesystem-dev")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_PACKAGE_VENDOR "Your Organization")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
include(CPack)