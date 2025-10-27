cmake_minimum_required(VERSION 2.8.3)

# Combined project (choose one name or make them co-exist)
project(feature_tracker_event)

# Global settings for both projects
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

#=============================
# TBB Configuration - Fix TBB path issue
#=============================
# Set TBB to use system installation instead of local ext/tbb
set(TBB_DIR "/usr/lib/x86_64-linux-gnu/cmake/TBB")
set(TBB_ROOT "/usr")

#=============================
# Better Flow Configuration
#=============================
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/better_flow_core/ext 
    ${CMAKE_CURRENT_SOURCE_DIR}/better_flow_core/ext_build
)

#=============================
# Common Dependencies
#=============================
find_package(catkin REQUIRED COMPONENTS 
    cv_bridge 
    geometry_msgs 
    image_transport 
    message_generation 
    pcl_conversions 
    pcl_ros 
    roscpp 
    sensor_msgs
    visualization_msgs 
    tf
    tf_conversions
    laser_geometry
    std_msgs
    dvs_msgs
    camera_model
    message_filters
)

find_package(OpenCV REQUIRED)
find_package(OpenCL)
find_package(Eigen3 REQUIRED)  # For feature_tracker
find_package(PCL REQUIRED)     # For better_flow

# OpenCL handling
if (OPENCL_FOUND)
    message(STATUS "OpenCL found: ${OpenCL_LIBRARIES} ${OpenCL_INCLUDE_DIRS}")
    add_definitions(-DOPENCL_ENABLED=0)
else()
    message(WARNING "OpenCL not found")
    set(OpenCL_LIBRARY "")
    set(OpenCL_INCLUDE_DIRS "")
endif()

catkin_package(CATKIN_DEPENDS 
    message_runtime 
    sensor_msgs 
    std_msgs 
    geometry_msgs 
    dvs_msgs
)

#=============================
# Include Directories
#=============================
include_directories(
    .
    include
    better_flow_core/include
    combiner/include
    ${TBB_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    /home/lxy/vins_mono_ws/src/VINS-Mono/feature_tracker/src/line_descriptor/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#=============================
# better_flow Executable
#=============================
add_executable(bf_visualizer_nd
    # Header files
    better_flow_core/include/better_flow/common.h
    better_flow_core/include/better_flow/event_file.h
    better_flow_core/include/better_flow/event.h
    better_flow_core/include/better_flow/clustering.h
    better_flow_core/include/better_flow/object_model.h
    better_flow_core/include/better_flow/optimizer_global.h
    better_flow_core/include/better_flow/optimizer_sampler.h
    better_flow_core/include/better_flow/optimizer_rolling.h
    better_flow_core/include/better_flow/opencl_driver.h
    better_flow_core/include/better_flow/accel_lib.h

    # Source files
    src/feature_tracker.cpp
    src/feature_tracker.h
    src/parameters.cpp
    src/parameters.h
    src/tic_toc.h
    src/bf_visualizer.cpp
    better_flow_core/src/event_file.cpp
    better_flow_core/src/clustering.cpp
    better_flow_core/src/object_model.cpp
    better_flow_core/src/optimizer_global.cpp
    better_flow_core/src/optimizer_sampler.cpp
    better_flow_core/src/optimizer_rolling.cpp
    better_flow_core/src/opencl_driver.cpp
)

target_link_libraries(bf_visualizer_nd
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${TBB_IMPORTED_TARGETS}
    tbb
    ${PCL_LIBRARIES}
    ${OpenCL_LIBRARY}
)
