cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

# OpenCV
find_package(OpenCV REQUIRED)
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Ceres
find_package(Ceres REQUIRED)

#siftgpu
set(SIFTGPU_INCLUDE_DIR "/home/mininx/Fast-Drone-250/src/realflight_modules/sift_vins_rtabmap/SiftGPU/src/SiftGPU")
link_directories("/home/mininx/Fast-Drone-250/src/realflight_modules/sift_vins_rtabmap/SiftGPU/bin")

# Eigen3
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${SIFTGPU_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

catkin_package()

add_library(vins_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${OpenCV_xfeatures2d_LIBS}
        siftgpu
        GLEW
        glut
        GL
        IL)


add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib) 

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test vins_lib) 

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test vins_lib) 

#set_target_properties(vins_node PROPERTIES
#        INSTALL_RPATH "/home/l/opencv_install/4.5.5/lib"
#        BUILD_WITH_INSTALL_RPATH TRUE
#)

