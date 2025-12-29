cmake_minimum_required(VERSION 3.0.2)
project(VINS-Fusion)

find_package(OpenCV 4.2 EXACT REQUIRED)
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
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
        ${OpenCV_INCLUDE_DIRS}  # 关键：使用find_package返回的路径
)

link_directories(${OpenCV_LIBRARY_DIRS})  # 使用OpenCV返回的库路径



set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# 标准 ROS 配置
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  
  cv_bridge
  geometry_msgs
  nav_msgs
  tf
  image_transport
  message_filters
)

# 关键：全局包含 OpenCV 头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # 确保此行存在
#        "/home/l/opencv_install/4.5.5/include/opencv4/"
)
#4,5,5
#link_directories(
#        "/home/l/opencv_install/4.5.5/lib/"
#        ${catkin_LIBRARY_DIRS}
#)
# 其他依赖...
catkin_package()

# 子目录
add_subdirectory(camera_models)
add_subdirectory(feature_tracker)
add_subdirectory(vins_estimator)
add_subdirectory(loop_fusion)
add_subdirectory(global_fusion)
