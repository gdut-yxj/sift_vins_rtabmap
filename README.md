# siftgpu + vins_fusion + rtabmap
camera_models、global_fusion、loop_fusion需要与原版的VINS_fusion进行替换

修改~/sift_vins_rtabmap/VINS-Fusion/vins_estimator路径下的CMakeLists.txt

set(SIFTGPU_INCLUDE_DIR "/home/dangeidon/vins_sws/src/SiftGPU/src/SiftGPU")
link_directories("/home/dangeidon/vins_sws/src/SiftGPU/bin")
修改为
set(SIFTGPU_INCLUDE_DIR "~/工作空间/src/SiftGPU/src/SiftGPU")
link_directories("~/工作空间/src/SiftGPU/bin")
