#ifndef VINS_RTABMAP_FUSION_GLOBAL_OPTIMIZER_HPP
#define VINS_RTABMAP_FUSION_GLOBAL_OPTIMIZER_HPP

// ROS 标准头文件
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

// RTAB-Map 核心库头文件 
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>

// C++ 标准库
#include <mutex>
#include <thread>
#include <memory>
#include <map>

namespace vins_rtabmap_fusion {

// 定义时间同步策略：包含 RGB, Depth, Info, Odom五路数据
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    nav_msgs::Odometry
> SyncPolicy;

class GlobalOptimizer {
public:
    GlobalOptimizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~GlobalOptimizer();

private:
    /**
     * @brief 核心回调函数，处理同步后的传感器数据
     */
    void processCallback(
        const sensor_msgs::ImageConstPtr& rgbMsg,
        const sensor_msgs::ImageConstPtr& depthMsg,
        const sensor_msgs::CameraInfoConstPtr& infoMsg,
        const nav_msgs::OdometryConstPtr& odomMsg
    );

    /**
     * @brief 辅助函数：将ROS位姿转换为RTAB-Map变换矩阵
     */
    rtabmap::Transform rosPoseToTransform(const geometry_msgs::Pose& pose);

    /**
     * @brief 辅助函数：将RTAB-Map变换矩阵转换为ROS变换信息
     */
    geometry_msgs::Transform rtabmapTransformToRos(const rtabmap::Transform& transform);

    /**
     * @brief 发布可视化话题 (Path, MapGraph)
     */
    void publishVisualization(
        const std::map<int, rtabmap::Transform>& poses, 
        const std::multimap<int, rtabmap::Link>& constraints,
        const ros::Time& stamp,
        const std::string& mapFrameId
    );

    /**
     * @brief 计算并发布 Map -> World 的TF变换
     */
    void publishTF(const rtabmap::Transform& mapToOdom, const ros::Time& stamp);

    // --- ROS 句柄 ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // --- 消息过滤器订阅者 ---
    message_filters::Subscriber<sensor_msgs::Image> subRGB_;
    message_filters::Subscriber<sensor_msgs::Image> subDepth_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> subInfo_;
    message_filters::Subscriber<nav_msgs::Odometry> subOdom_;

    // --- 同步器 ---
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // --- 发布者 ---
    ros::Publisher pubGlobalPath_;
    ros::Publisher pubMapGraph_; 
    ros::Publisher pubGlobalOdom_;

    // --- TF 广播 ---
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    // --- RTAB-Map 核心对象 ---
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;
    
    // --- 状态变量 ---
    std::string mapFrameId_;   // 世界坐标系，默认 "map"
    std::string odomFrameId_;  // 里程计坐标系，默认 "world" 
    std::string robotFrameId_; // 机器人坐标系，默认 "base_link" (或 "body")
    
    rtabmap::Transform mapToOdom_; // 累积漂移修正矩阵
    std::mutex mapToOdomMutex_;    // 保护 TF 更新的互斥锁
    
    bool pauseMapping_;
};

} 

#endif // VINS_RTABMAP_FUSION_GLOBAL_OPTIMIZER_HPP