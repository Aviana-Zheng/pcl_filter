# Basic usage, [Using PCL in your own project](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pcl_filter)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcl_filter pcl_filter.cpp)
target_link_libraries(pcl_filter ${PCL_LIBRARIES})
add_custom_command(TARGET pcl_filter PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/resources/1.pcd ${PROJECT_BINARY_DIR}/
)