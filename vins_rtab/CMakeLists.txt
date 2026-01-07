cmake_minimum_required(VERSION 3.0.2)
project(vins_rtab)

# 查找必要的 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  message_filters
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  rtabmap_ros        # 用于消息定义和部分辅助转换
  rtabmap_conversions # 关键：ROS消息到RTAB-Map类型的转换工具
  rtabmap_msgs
  pcl_conversions
  pcl_ros
)

# 查找 RTAB-Map 独立库 (Core Library)
# 注意：这会在系统中查找 rtabmap 安装，而非 catkin 工作空间中的 rtabmap_ros
find_package(RTABMap REQUIRED)

# 查找 Eigen3 数学库
find_package(Eigen3 REQUIRED)

# 声明 catkin 包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vins_rtabmap_fusion
  CATKIN_DEPENDS roscpp rtabmap_ros message_filters cv_bridge tf2_ros
)

# 包含路径配置
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RTABMap_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 声明节点可执行文件
add_executable(global_optimizer_node 
  src/GlobalOptimizer.cpp 
  src/main.cpp
)

# 链接库文件
target_link_libraries(global_optimizer_node
  ${catkin_LIBRARIES}
  ${RTABMap_LIBRARIES} # 链接 librtabmap_core.so, librtabmap_utilite.so 等
)