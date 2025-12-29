cmake_minimum_required(VERSION 2.8.3)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
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
        ${OpenCV_INCLUDE_DIRS}  # 使用CMake返回的正确路径
        include  # 保留本地头文件路径
)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

find_package(OpenCV 4.2 EXACT REQUIRED)
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")




find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})


# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_models
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )



include_directories("include")

add_executable(Calibrations 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

add_library(camera_models
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

