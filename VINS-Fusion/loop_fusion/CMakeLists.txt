cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#set(OpenCV_DIR "/home/l/opencv_install/4.5.5/lib/cmake/opencv4")
#find_package(OpenCV 4.5.5 EXACT REQUIRED
#        NO_DEFAULT_PATH
#        NO_CMAKE_ENVIRONMENT_PATH
#        NO_CMAKE_PATH
#        NO_SYSTEM_ENVIRONMENT_PATH
#        NO_CMAKE_SYSTEM_PATH
#)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}  # 使用CMake返回的标准路径
        ${EIGEN3_INCLUDE_DIR}
)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_models
    cv_bridge
    roslib
    )


find_package(OpenCV 4.2 EXACT REQUIRED)
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

#link_directories("/home/l/opencv_install/4.5.5/lib/")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
#set_target_properties(loop_fusion_node PROPERTIES
#        INSTALL_RPATH "/home/l/opencv_install/4.5.5/lib"
#        BUILD_WITH_INSTALL_RPATH TRUE
#)
