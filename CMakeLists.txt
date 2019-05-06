cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
project(extract_indices)
SET(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.7 REQUIRED)
#set(OpenCV_DIR /usr/local/opencv2/share/OpenCV)
FIND_PACKAGE( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(
include
)

add_library( dbscan_shared SHARED dbscan.cpp )
add_executable (dbscan_main dbscan_main.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

add_executable (main_pipeline main_pipeline.cpp)
target_link_libraries (main_pipeline ${PCL_LIBRARIES} ${OpenCV_LIBS} dbscan_shared)

add_executable (adjust_gt_intensity adjust_gt_intensity.cpp)
target_link_libraries (adjust_gt_intensity ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (draw_walledge draw_walledge.cpp)
target_link_libraries (draw_walledge ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable (region_growing region_growing.cpp)
# target_link_libraries (region_growing ${PCL_LIBRARIES})

#add_executable (supervoxel_lccp supervoxel_lccp.cpp)
#target_link_libraries (supervoxel_lccp ${PCL_LIBRARIES})

# add_executable (mincut_segmentation mincut_segmentation.cpp)
# target_link_libraries (mincut_segmentation ${PCL_LIBRARIES})
# 
# add_executable ( DoN_segmentation DoN_segmentation.cpp )
# target_link_libraries (DoN_segmentation ${PCL_LIBRARIES})
# 
# add_executable (zhang_calibration zhang_calibration.cpp)
# target_link_libraries (zhang_calibration ${OpenCV_LIBS})
# 
# add_executable (delaunay2 delaunay2.cpp)
# target_link_libraries (delaunay2 ${OpenCV_LIBS})

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})


target_link_libraries (dbscan_main ${PCL_LIBRARIES} dbscan_shared)

