# SiftGPU + VINS-Fusion + RTAB-Map

## 使用注意事项

### 1. 编译 SiftGPU

如果使用 **ARM64 架构**，需要修改默认的 x86 编译参数：

- 打开 `Fast-Drone-250/src/realflight_modules/sift_vins_rtabmap/SiftGPU/makefile`
- 找到以下两处：
  ```makefile
  siftgpu_sse_options = -march=core2 -mfpmath=sse
  ```
- 修改为：
  ```makefile
  siftgpu_sse_options =
  ```
  即删除 `-march=core2 -mfpmath=sse` 参数。

- 修改后重新编译：
  ```bash
  cd ~/Fast-Drone-250/src/realflight_modules/sift_vins_rtabmap/SiftGPU
  make clean
  make -j$(nproc)
  ```

### 2. 修改 VINS-Fusion 的 CMakeLists.txt

路径：`~/sift_vins_rtabmap/VINS-Fusion/vins_estimator/CMakeLists.txt`

- 原设置：
  ```cmake
  set(SIFTGPU_INCLUDE_DIR "/home/dangeidon/vins_sws/src/SiftGPU/src/SiftGPU")
  link_directories("/home/dangeidon/vins_sws/src/SiftGPU/bin")
  ```

- 修改为：
  ```cmake
  set(SIFTGPU_INCLUDE_DIR "~/工作空间/src/SiftGPU/src/SiftGPU")
  link_directories("~/工作空间/src/SiftGPU/bin")
  ```

---

## 使用步骤

1. **开启相机**
   ```bash
   roslaunch realsense2_camera rs_camera.launch
   ```

2. **打开 RViz**
   ```bash
   roslaunch vins vins_rviz.launch
   ```

3. **启动 VINS 节点并加载参数文件**
   ```bash
   rosrun vins vins_node ~/vins_sws/src/VINS-Fusion/config/3.5_drone/realsense_stereo_imu_config.yaml
   ```

4. **启动回环节点**
   ```bash
   rosrun vins_rtab global_optimizer_node
   ```

---

## 话题观察

- **World 坐标系下 body 位姿**：
  ```bash
  rostopic echo /vins_estimator/odometry
  ```

- **Map 坐标系下 body 位姿**：
  ```bash
  rostopic echo /rtabmap/global_odom
  ```
```
