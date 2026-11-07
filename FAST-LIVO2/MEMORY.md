# 内存修复与运行配置

本版本修复 FAST-LIVO2 ROS 2 移植中的运行期泄漏，并为在线数据设置保留策略。

## 资源管理与代码风格

- Feature 使用 unique_ptr<float[]> 拥有图像块，只有新增观测时才分配。
- Warp 按值缓存，clear() 同时销毁对象。
- 视觉体素由 unique_ptr 管理；地图淘汰只在帧边界执行，先清理借用指针。
- 视觉点仍拥有其 Feature 列表，局部子图只借用 VisualPoint。
- 相机改成单一所有者；激光地图析构递归释放八叉树。
- Vikit 与 FAST-LIVO2 都使用 C++17，避免 Eigen 对齐分配/释放跨标准不一致。
- .clang-format 基于 Google，保留本项目的 Allman 大括号、两空格缩进和右侧指针星号。
  沿用已有函数命名；新增独立工具遵循 Google 命名，避免全仓重命名。

## 默认参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| memory.max_lidar_buffer | 30 | 雷达应用队列和 DDS 接收历史深度 |
| memory.max_image_buffer | 10 | 图像应用队列和 DDS 接收历史深度 |
| memory.max_imu_buffer | 1000 | IMU 和预测缓存数量上限 |
| memory.max_buffer_duration | 3.0 | 每种传感器缓存的最大时间跨度，秒 |
| memory.max_path_poses | 10000 | 发布轨迹的最大位姿数量 |
| memory.max_visual_points | 20000 | 帧边界清理后的视觉地图点上限 |
| memory.max_visual_points_per_voxel | 32 | 每个视觉体素的地图点上限 |
| memory.max_reference_frames | 200 | 视觉观测图像的最大历史帧跨度 |
| memory.visual_map_radius | 200.0 | 相对当前相机的视觉地图半径，米 |
| pcd_save.interval | 100 | 分块保存间隔；默认 pub_scan_num=1 时按发布帧计数 |
| pcd_save.max_points | 2000000 | 每个保存块的点数上限，单帧过大时拆成多个文件 |
| pcd_save.output_directory | 源码目录/Log/PCD | PCD 文件目录 |

传感器缓存数量、持续时间、路径长度和分块点数必须为正数。
视觉地图的四个保留参数可以分别设为 0，关闭对应限制。
地图点上限在帧边界执行，单帧新插入的地图点可能临时超过该数量。
RGB/灰度图像缓存、点云单帧大小和分配器容量仍会影响峰值 RSS；
这些参数不是进程级的固定字节数配额。

两个现有传感器配置默认开启激光地图滑窗。local_map.half_map_size 的单位
仍为体素数，不是米；Avia 的 voxel_size=0.5、half_map_size=100
对应每轴半宽约 50 m，MARS 的 voxel_size=2.0 时对应约 200 m。
激光滑窗和视觉地图保留策略分别生效。离开范围的在线地图会被淘汰。

队列满或时间跨度过大时，丢弃最旧数据，并重置尚未完成的测量同步，
同时输出节流告警。传感器配置缺失、长期时间不同步或处理速度不足时，
应先修正数据源或降低回放速度；丢帧后的轨迹精度需要重新评估。

MARS 启动入口同时加载雷达/相机参数，并统一相机参数到 camera.* 命名空间。

原始图像使用 toCvCopy 获得独立像素所有权，保证 ROS 消息释放后排队图像仍有效。
订阅使用 SensorDataQoS（best-effort/volatile），兼容 reliable 和 best-effort 发布端。

## 点云与可视化

- 默认使用分块保存，退出时保存最后不足一块的数据。
- 达到 interval 或 max_points 任一条件即写盘；写盘失败不清空待保存点云，
  并停止节点，避免无限缓存或静默丢失。
- 连续编号文件保存的是各时段的全局坐标点云，完整地图可离线合并。
- 显式设置 interval=-1 恢复整段点云留在内存中的旧行为，
  这时 max_points 不生效；COLMAP 导出仍要求这个模式。
- filter_size_pcd 只用于整段保存模式的最终降采样，不影响实时缓存容量。
- RViz 的 FAST-LIVO2 和 M300 预设把历史点云保留时间缩短为最多 10 秒。
  要保存完整建图结果，请使用 PCD 文件，而不是让 RViz 长期缓存所有帧。

## 构建

本工作区原 build/ 中有迁移前路径缓存，因此使用独立构建目录：

~~~bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/local_setup.bash
cd /home/ros/rosws/livo_algo_ws
PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=6 colcon build \
  --packages-up-to fast_livo --build-base build_memory_fix \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
~~~

必须同时重建工作区内的 vikit_common；不要混用此前按 C++11 编译的相机库。

## 回归测试与泄漏检查

~~~bash
colcon test --packages-select fast_livo --build-base build_memory_fix \
  --return-code-on-test-failure
colcon test-result --test-result-base build_memory_fix/fast_livo --verbose

# 先完成上面的 Release 构建，确保安装的 Vikit 已统一为 C++17。
PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build \
  --packages-select fast_livo --build-base build_memory_asan \
  --install-base install_memory_asan --cmake-args \
  -DFAST_LIVO_ENABLE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS_DEBUG=-O1 -g1'
colcon test --packages-select fast_livo --build-base build_memory_asan \
  --install-base install_memory_asan --return-code-on-test-failure
colcon test-result --test-result-base build_memory_asan/fast_livo --verbose
~~~

Sanitizer 模式禁用 mimalloc。test/lsan.supp 仅排除系统旧 libtbb.so.2 的固定
进程期分配，不排除项目代码。测试覆盖无新增观测时的图像块、观测替换、
地图容量/时效/空间淘汰、悬空别名清理、长期地图增长、Warp 缓存释放、
递归八叉树释放、传感器积压、时间回退、图像像素生命周期、纯 LIO 测量组、
PCD 分块与失败处理、空 RGB 投影及路径长度。

真实数据集上的定位精度与吞吐量仍需使用相同 bag 做前后对比；
缓存溢出告警、进程 RSS 和 RViz RSS 应分别记录。
