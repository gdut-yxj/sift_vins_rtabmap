#include "vins_rtab/GlobalOptimizer.hpp"

// ROS 工具库
#include <cv_bridge/cv_bridge.h>
#include <rtabmap_conversions/MsgConversion.h> 
#include <rtabmap_msgs/MapGraph.h>
#include <rtabmap_msgs/MapData.h>

// RTAB-Map 工具库
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UException.h> // 引入异常头文件

namespace vins_rtabmap_fusion {

GlobalOptimizer::GlobalOptimizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), mapToOdom_(rtabmap::Transform::getIdentity())
{
    // 参数配置
    pnh_.param<std::string>("map_frame_id", mapFrameId_, "map");
    pnh_.param<std::string>("odom_frame_id", odomFrameId_, "world");
    pnh_.param<std::string>("robot_frame_id", robotFrameId_, "body");
    
    bool deleteDbOnStart;
    pnh_.param<bool>("delete_db_on_start", deleteDbOnStart, true); 

    std::string dbPath;
    pnh_.param("database_path", dbPath, UDirectory::homeDir() + "/.ros/rtabmap.db");

    // 初始化 RTAB-Map 参数
    rtabmap::ParametersMap parameters;
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "true"));
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapLoopThr(), "0.20")); 
    
    // 关闭邻接边重优化，完全信任 VINS 里程计
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDNeighborLinkRefining(), "false")); 
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDProximityBySpace(), "false")); 
    
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), "0")); 
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
    
    // 优化器配置
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerStrategy(), "1")); // 1=g2o
    
    // 日志设置
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    // 数据库清理
    if (deleteDbOnStart) {
        if (UFile::exists(dbPath)) {
            ROS_WARN("Deleting old database: %s", dbPath.c_str());
            UFile::erase(dbPath);
        }
    }

    rtabmap_ = std::make_unique<rtabmap::Rtabmap>();
    rtabmap_->init(parameters, dbPath);

    // 设置发布者
    pubGlobalPath_ = nh_.advertise<nav_msgs::Path>("/rtabmap/global_path", 1);
    pubMapGraph_ = nh_.advertise<rtabmap_msgs::MapGraph>("/rtabmap/mapGraph", 1);
    pubGlobalOdom_ = nh_.advertise<nav_msgs::Odometry>("/rtabmap/global_odom", 1);  
    
    // 订阅与同步
    subRGB_.subscribe(nh_, "/camera/color/image_raw", 1);
    subDepth_.subscribe(nh_, "/camera/aligned_depth_to_color/image_raw", 1);
    subInfo_.subscribe(nh_, "/camera/color/camera_info", 1);
    subOdom_.subscribe(nh_, "/vins_estimator/odometry", 1);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), subRGB_, subDepth_, subInfo_, subOdom_
    );

    sync_->registerCallback(boost::bind(&GlobalOptimizer::processCallback, this, _1, _2, _3, _4));

    ROS_INFO("GlobalOptimizer initialized. Listening for VINS data...");
}

GlobalOptimizer::~GlobalOptimizer() {
    if(rtabmap_) {
        rtabmap_->close(true);
    }
}

void GlobalOptimizer::processCallback(
    const sensor_msgs::ImageConstPtr& rgbMsg,
    const sensor_msgs::ImageConstPtr& depthMsg,
    const sensor_msgs::CameraInfoConstPtr& infoMsg,
    const nav_msgs::OdometryConstPtr& odomMsg)
{
    ros::Time stamp = rgbMsg->header.stamp;

    // --- 数据转换 ---
    cv::Mat rgb, depth;
    try {
        rgb = cv_bridge::toCvShare(rgbMsg, "bgr8")->image;
        depth = cv_bridge::toCvShare(depthMsg)->image; 
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*infoMsg);
    rtabmap::Transform odomPose = rosPoseToTransform(odomMsg->pose.pose);
    rtabmap::SensorData data(rgb, depth, model, 0, stamp.toSec());

    // --- 协方差输入保护 (防止输入端崩溃) ---
    // 强制对角化 + 限幅
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64F); 
    const auto& raw_cov = odomMsg->pose.covariance;
    
    double min_cov = 1e-6; 
    double max_cov = 0.5;  

    for(int i = 0; i < 6; ++i) {
        double val = raw_cov[i*6 + i]; 
        if(std::isnan(val) || std::isinf(val)) val = max_cov;
        else if(val < min_cov) val = min_cov;
        else if(val > max_cov) val = max_cov;
        covariance.at<double>(i, i) = val;
    }

    // --- 执行 RTAB-Map 处理 ---
    // 使用 try-catch 包裹，因为 process 内部也可能抛出异常
    bool processed = false;
    try {
        processed = rtabmap_->process(data, odomPose, covariance);
    } catch (const UException& e) {
        ROS_ERROR("RTAB-Map process error: %s", e.what());
        return;
    }

    // --- D. 后处理与可视化 ---
    if (processed) {
            int loopId = rtabmap_->getLoopClosureId();
            
            if (loopId > 0) {
                ROS_INFO("Loop closure detected: %d -> %d. Updating Map Correction!", 
                    rtabmap_->getLastLocationId(), loopId);

                std::map<int, rtabmap::Transform> poses;
                std::multimap<int, rtabmap::Link> constraints;
                
                try {
                    // 获取优化后的图
                    rtabmap_->getGraph(poses, constraints, true, true);
                    
                    if (!poses.empty()) {
                        // 获取当前最新帧在 map 下的优化位姿
                        // 注意：这里要确保取到的是当前处理的这一帧的ID
                        int currentId = rtabmap_->getLastLocationId();
                        if (poses.find(currentId) != poses.end()) {
                            rtabmap::Transform mapPose = poses.at(currentId);
                            
                            // 计算修正量 T_map_world = T_map_body * T_world_body^-1
                            // 这样 T_map_world * T_world_body = T_map_body
                            rtabmap::Transform mapToOdomCorrection = mapPose * odomPose.inverse();

                            {
                                std::lock_guard<std::mutex> lock(mapToOdomMutex_);
                                mapToOdom_ = mapToOdomCorrection;
                            }
                        }
                    }
                } catch (const UException& e) {
                    ROS_WARN("Graph optimization failed during loop closure, skipping correction update.");
                }
            }
            
            // 只有当有订阅者时才去获取图并发布
            if (pubGlobalPath_.getNumSubscribers() > 0 || pubMapGraph_.getNumSubscribers() > 0 || pubGlobalOdom_.getNumSubscribers() > 0) {
                std::map<int, rtabmap::Transform> poses;
                std::multimap<int, rtabmap::Link> constraints;
                try {
                    rtabmap_->getGraph(poses, constraints, true, true);
                    publishVisualization(poses, constraints, stamp, mapFrameId_);
                } catch(...) {}
            }
        }

        {
            std::lock_guard<std::mutex> lock(mapToOdomMutex_);
            
            // 发布 TF (map -> world)
            publishTF(mapToOdom_, stamp);

            // 发布全局里程计 (map -> body)
            // GlobalPose = Correction * VinsPose
            // T_map_body = T_map_world * T_world_body
            rtabmap::Transform globalPose = mapToOdom_ * odomPose;

            nav_msgs::Odometry globalOdomMsg;
            globalOdomMsg.header.stamp = stamp;
            globalOdomMsg.header.frame_id = mapFrameId_; // "map"
            globalOdomMsg.child_frame_id = robotFrameId_; // "body"
            
            // 转换 Pose
            geometry_msgs::Transform tRos = rtabmapTransformToRos(globalPose);
            globalOdomMsg.pose.pose.position.x = tRos.translation.x;
            globalOdomMsg.pose.pose.position.y = tRos.translation.y;
            globalOdomMsg.pose.pose.position.z = tRos.translation.z;
            globalOdomMsg.pose.pose.orientation = tRos.rotation;

            // 填充协方差 (简单复制 VINS 协方差，虽然不严谨但够用)
            globalOdomMsg.pose.covariance = odomMsg->pose.covariance;

            pubGlobalOdom_.publish(globalOdomMsg);
        }
}

void GlobalOptimizer::publishVisualization(
    const std::map<int, rtabmap::Transform>& poses, 
    const std::multimap<int, rtabmap::Link>& constraints,
    const ros::Time& stamp,
    const std::string& mapFrameId)
{
    // 1. 发布 Path
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = mapFrameId;
    pathMsg.header.stamp = stamp;

    for(const auto& iter : poses) {
        geometry_msgs::PoseStamped ps;
        ps.header = pathMsg.header;
        geometry_msgs::Transform tRos = rtabmapTransformToRos(iter.second);
        ps.pose.position.x = tRos.translation.x;
        ps.pose.position.y = tRos.translation.y;
        ps.pose.position.z = tRos.translation.z;
        ps.pose.orientation = tRos.rotation;
        pathMsg.poses.push_back(ps);
    }
    pubGlobalPath_.publish(pathMsg);

    // 2. 发布 MapGraph (Link)
    // 只有当有人订阅时才计算，节省资源 (且这就是你之前崩溃的触发点)
    if(pubMapGraph_.getNumSubscribers() > 0) {
        rtabmap_msgs::MapGraph graphMsg;
        graphMsg.header = pathMsg.header;
        graphMsg.mapToOdom = rtabmapTransformToRos(mapToOdom_);
        
        graphMsg.posesId.resize(poses.size());
        graphMsg.poses.resize(poses.size());
        int i=0;
        for(auto iter=poses.begin(); iter!=poses.end(); ++iter, ++i){
            graphMsg.posesId[i] = iter->first;
            geometry_msgs::Transform tRos = rtabmapTransformToRos(iter->second);
            graphMsg.poses[i].position.x = tRos.translation.x;
            graphMsg.poses[i].position.y = tRos.translation.y;
            graphMsg.poses[i].position.z = tRos.translation.z;
            graphMsg.poses[i].orientation = tRos.rotation;
        }

        graphMsg.links.resize(constraints.size());
        i=0;
        for(auto iter=constraints.begin(); iter!=constraints.end(); ++iter, ++i){
            rtabmap_msgs::Link linkMsg;
            linkMsg.fromId = iter->second.from();
            linkMsg.toId = iter->second.to();
            linkMsg.type = iter->second.type();
            linkMsg.transform = rtabmapTransformToRos(iter->second.transform());
            
            // 【核心修复：可视化协方差填充】
            // 必须手动填充 information matrix，不能留 0
            // 我们从 RTAB-Map 的 link 中读取 matrix
            cv::Mat info = iter->second.infMatrix();
            
            // 检查 matrix 是否有效
            if(info.empty() || info.cols != 6 || info.rows != 6) {
                // 如果无效，给一个默认单位矩阵
                for(int k=0; k<36; ++k) linkMsg.information[k] = (k%7==0)? 1.0 : 0.0;
            } else {
                // 复制数据，同时检查是否为 0
                bool allZeros = true;
                const double* ptr = (const double*)info.data;
                for(int k=0; k<36; ++k) {
                    linkMsg.information[k] = ptr[k];
                    if(ptr[k] != 0) allZeros = false;
                }
                // 如果全是 0 (比如 RTAB-Map 没算出来)，Rviz 收到会崩
                // 强制设为单位矩阵
                if(allZeros) {
                    for(int k=0; k<36; ++k) linkMsg.information[k] = (k%7==0)? 1.0 : 0.0;
                }
            }

            graphMsg.links[i] = linkMsg;
        }
        pubMapGraph_.publish(graphMsg);
    }
}

rtabmap::Transform GlobalOptimizer::rosPoseToTransform(const geometry_msgs::Pose& pose) {
    return rtabmap::Transform(
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    );
}

geometry_msgs::Transform GlobalOptimizer::rtabmapTransformToRos(const rtabmap::Transform& tr) {
    geometry_msgs::Transform t;
    t.translation.x = tr.x();
    t.translation.y = tr.y();
    t.translation.z = tr.z();
    Eigen::Quaternionf q = tr.getQuaternionf();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();
    return t;
}

void GlobalOptimizer::publishTF(const rtabmap::Transform& mapToOdom, const ros::Time& stamp) {
    geometry_msgs::TransformStamped tfStamped;
    tfStamped.header.stamp = stamp;
    tfStamped.header.frame_id = mapFrameId_;   
    tfStamped.child_frame_id = odomFrameId_;   
    tfStamped.transform = rtabmapTransformToRos(mapToOdom);
    tfBroadcaster_.sendTransform(tfStamped);
}

} // namespace vins_rtabmap_fusion