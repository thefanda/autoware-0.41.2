# YabLoc 使用与运行梳理（autoware-0.41.2）

本文档基于当前工作空间源码整理，目标是说明：
- YabLoc 是什么
- 如何启动
- 运行时输入输出
- 端到端数据流
- 常见检查与排障方法

## 1. YabLoc 是什么

YabLoc 是 Autoware 中的视觉 + 矢量地图定位模块。  
它使用相机图像提取路面线段，并和 Lanelet2 地图要素做匹配，通过粒子滤波估计车辆位姿。

在本工程中，YabLoc 作为 `pose_source` 的一种选择，与 `ndt`、`eagleye` 等并列。

参考：
- `src/universe/autoware.universe/localization/yabloc/README.md`
- `src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml`

## 2. 如何启动

## 2.1 推荐方式：通过 autoware_launch 启动全栈

将 `pose_source` 指定为 `yabloc`：

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  pose_source:=yabloc
```

说明：
- `pose_source` 默认通常是 `ndt`，改成 `yabloc` 后会走 YabLoc 定位链路。
- 该命令来自 YabLoc README 的示例。

## 2.2 启动链路（源码对应）

1. `autoware_launch` 读取 `pose_source`
2. 进入 `tier4_localization_launch/localization.launch.xml`
3. 在 `pose_twist_estimator.launch.xml` 中判断 `use_yabloc_pose`
4. include `yabloc.launch.xml`

关键文件：
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml`
- `src/universe/autoware.universe/launch/tier4_localization_launch/launch/localization.launch.xml`
- `src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml`
- `src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/yabloc.launch.xml`

## 3. YabLoc 启动后会拉起哪些模块

`yabloc.launch.xml` 下有 5 组子模块：

1. `initializer`
- 包：`yabloc_pose_initializer`
- 作用：给初始位姿候选（依赖模型文件）

2. `pf`（particle filter）
- 包：`yabloc_particle_filter`
- 作用：粒子预测 + 相机/GNSS 校正，输出最终位姿

3. `image_processing`
- 包：`yabloc_image_processing`
- 作用：去畸变、线段提取、路面分割、线段投影、可视化叠加

4. `map`
- 包：`yabloc_common`
- 作用：Lanelet2 分解（road_marking/sign_board/bounding_box）和地面参数估计

5. `monitor`
- 包：`yabloc_monitor`
- 作用：监控 YabLoc 位姿输出并发布诊断

## 4. 运行输入（必须重点确认）

## 4.1 关键输入 topic

- 相机图像：`/sensing/camera/traffic_light/image_raw`
- 相机内参：`/sensing/camera/traffic_light/camera_info`
- 矢量地图：`/map/vector_map`
- 预测速度输入：`/localization/twist_estimator/twist_with_covariance`
- GNSS（常用）：`/sensing/gnss/pose_with_covariance`
- 初始位姿：`/initialpose3d`

## 4.2 关键前提

- 存在从 `base_link` 到相机光学坐标系的 `tf_static`
- pose initializer 模型文件存在（默认）：
  - `$HOME/autoware_data/yabloc_pose_initializer/saved_model/model_float32.pb`

## 5. 运行输出（最关键）

## 5.1 最终定位输出

- `/localization/pose_estimator/pose_with_covariance`

这是 YabLoc 粒子滤波预测器 remap 后输出的核心结果，后级融合/规划主要看它。

## 5.2 常用调试输出

- `/localization/pose_estimator/yabloc/pf/pose`
- `/localization/pose_estimator/yabloc/pf/predicted_particles_marker`
- `/localization/pose_estimator/yabloc/pf/scored_cloud`
- `/localization/pose_estimator/yabloc/image_processing/lanelet2_overlay_image`
- `/localization/pose_estimator/yabloc/image_processing/segmented_image`
- `/diagnostics`

## 5.3 常用服务

- `/localization/pose_estimator/yabloc/initializer/yabloc_align_srv`
- `/localization/pose_estimator/yabloc/pf/yabloc_trigger_srv`

## 6. 端到端数据流（按处理顺序）

1. `undistort` 节点接收相机图像/内参，输出去畸变图像
2. `line_segment_detector` 提取线段
3. `graph_segment` 做路面区域分割
4. `segment_filter` 结合分割结果筛选并投影线段
5. `ll2_decomposer` 将 Lanelet2 地图拆成路面标线等点云
6. `ground_server` 输出地面参数与高度
7. `predictor` 根据速度与高度预测粒子
8. `camera_particle_corrector` 用图像线段和地图匹配给粒子加权
9. `gnss_particle_corrector` 结合 GNSS 再加权如何启动
10. `predictor` 汇总加权粒子并输出 `pose_with_covariance`

## 7. 快速检查命令

```bash
ros2 topic hz /localization/pose_estimator/pose_with_covariance
ros2 topic echo /localization/pose_estimator/yabloc/pf/pose --once
ros2 topic hz /localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud
ros2 topic hz /localization/pose_estimator/yabloc/map/ll2_road_marking
ros2 topic echo /diagnostics --once
ros2 service list | grep yabloc
```

## 8. 常见问题与排障

1. 相机 topic 名称不一致
- 症状：图像链路无输出
- 处理：检查 `src_image/src_info` remap 是否对应真实 topic

2. TF 缺失或 frame 不匹配
- 症状：投影异常、定位漂移、overlay 对不上
- 处理：检查 `tf_static` 中 `base_link -> camera_optical_frame`

3. `/map/vector_map` 未发布
- 症状：地图相关节点无有效输出（ll2/ground）
- 处理：确认 map loader 正常发布 Lanelet2

4. initializer 模型文件缺失
- 症状：`yabloc_pose_initializer` 报错或服务不可用
- 处理：确认 `$HOME/autoware_data/yabloc_pose_initializer/saved_model/model_float32.pb` 存在

5. GNSS 异常影响粒子权重
- 症状：定位跳变或不稳定
- 处理：先单独检查相机链路与 GNSS 质量，再调整 GNSS corrector 参数

## 9. 关键源码索引

- 总览文档：
  - `src/universe/autoware.universe/localization/yabloc/README.md`
- 总 launch：
  - `src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/yabloc.launch.xml`
- 子模块 launch：
  - `src/universe/autoware.universe/localization/yabloc/yabloc_pose_initializer/launch/yabloc_pose_initializer.launch.xml`
  - `src/universe/autoware.universe/localization/yabloc/yabloc_particle_filter/launch/yabloc_particle_filter.launch.xml`
  - `src/universe/autoware.universe/localization/yabloc/yabloc_image_processing/launch/yabloc_image_processing.launch.xml`
  - `src/universe/autoware.universe/localization/yabloc/yabloc_common/launch/yabloc_common.launch.xml`
  - `src/universe/autoware.universe/localization/yabloc/yabloc_monitor/launch/yabloc_monitor.launch.xml`

