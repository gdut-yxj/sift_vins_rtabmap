#include "vins_rtab/GlobalOptimizer.hpp"

int main(int argc, char** argv) {
    // 初始化 RTAB-Map 日志系统，输出到控制台
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    ros::init(argc, argv, "vins_loop_fusion_replacement");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // 私有句柄，用于读取 <param>

    vins_rtabmap_fusion::GlobalOptimizer optimizer(nh, pnh);

    // 使用多线程 Spin 以防回调处理过慢（虽然 RTAB-Map 内部处理是同步的）
    // 对于图像处理回调，单线程 Spin 也可以，但建议分离
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    
    ros::waitForShutdown();

    return 0;
}