# 障碍处理建议核验与现场观察

核验范围：Workbench 基线 `c96d7f2`，`dependencies/field_test.repos` 锁定的
`agt_navigation_v3@b3a60487dff973c6326aca2ca2e9db1e78daeaaa`，以及仓库保存的
[10 月 25 日局部障碍清理证据](../costmap_clearing_20261025/README.md)。
本轮未连接实车、未重跑真实 Nav2 插件。云端未安装 ROS；以下历史实验结果不是本轮现场验收。

本仓库锁定的是 AGT 源码版本，**没有锁定现场 Nav2 deb 包版本**。参数语义另对照官方
Humble 源码快照 `3c3db59d6969d8ecee8e68468693d006397f4a0c`，不能据此声称现场已运行同一构建。

| 建议 | 判断 | 处理 |
| --- | --- | --- |
| `observation_persistence: 0.5` 能在人离开后强制清 voxel | 错误。这是观测缓存时长，不是栅格占据的 TTL | 保留已有 `0.0`；不增加此参数 |
| `stop_zone.max_points: 0 → 3~5` 可消除误停 | 缺少本仓库适用对象；即使部署了 Humble Collision Monitor，也会忽略更多真实稀疏障碍 | 不修改；先查实际停止节点和原因 |
| 连续 3 帧才保留障碍、0.5 秒自动消退 | 属于另一个需验证的感知策略，不能当作通用救急补丁 | 不进入当前人员障碍判定链 |
| RViz 展示过滤云、标记结果、保护边界 | 观察思路合理，需使用实际存在的输出 | 增加真实源的只读显示，不虚构 VoxelLayer/Stop Zone |

## 参数为何不支持所声称的效果

官方 [ObservationBuffer::purgeStaleObservations()](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/src/observation_buffer.cpp#L187)
只删除缓存的 Observation；`0.0` 保留最新一帧，正数保留指定时间窗的观测。
它不调用栅格清除。旧障碍仍需有效射线或其他明确清除机制才能从地图移除。
[ObstacleLayer::updateBounds()](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/plugins/obstacle_layer.cpp#L442)
先 raytrace clearing、再 marking；将标记观测缓存增至 0.5 秒，反而可能重复标记旧人体位置。

当前配置入口是 `profiles.navigation()` 合并 AGT 的
`robot/navigation/controller/costmap/perception/safety.yaml`，并非建议中的 `nav2_bunker.yaml`。
上游 `perception.yaml` 的源名为 `lidar3d_mark` 和 `lidar3d_clear`，并非 `obstacle_cloud`。
Workbench 已在 Bunker 生成配置中将 `voxel_layer` 换为原生 `ObstacleLayer`；
`/agt/navigation/points_obstacles` 用于标记，`/agt/livox/points` 用于二维射线清除。
配置、锁定依赖和启动入口中没有找到 `collision_monitor` 或 `stop_zone`。
目前可确认的控制保护链包括 Nav2 RPP 碰撞检测、速度平滑、`agt_cmd_vel_guard`；
不能把一次停止直接归因于并未配置的 Collision Monitor。

Humble 官方 [processStopSlowdown()](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_collision_monitor/src/collision_monitor_node.cpp#L411)
以 `points_inside > max_points` 触发：0 对应至少 1 点，3 对应至少 4 点，5 对应至少 6 点。
因此增加阈值会同时忽略一定数量的真实点；仅凭“可能是噪声”不能证明此修改可接受。
还必须先确认实际发行版和包版本，不能套用其他版本的参数名或比较规则。

当前 `agt_pointcloud_preprocessor` 每帧创建 `occupied` 体素集合，仅做单帧降采样；
没有 `min_consecutive_hits`、`max_age_sec`、`miss_decay` 参数。
增加时序过滤必须在共同坐标系补偿车体运动，并评估遮挡、稀疏回波、急入视场和断流。
现场历史记录障碍云约 5.51 Hz；等待第 3 帧最早增加约 `2/5.51 ≈ 0.36 s` 的确认延迟，
不能从这几个参数推出“人走后 0.2–0.3 秒恢复”。移动人员只出现在一个体素 1–2 帧
也可能是真实近距离障碍；把它过滤掉会削弱保护。感知点消失也不等于代价地图已清除。

## 已有修复证据

[隔离实验结果](../costmap_clearing_20261025/isolated_result.json)记录真实 Nav2 插件在合成输入下的对照：

| 场景 | 旧 VoxelLayer | 现用二维 ObstacleLayer |
| --- | ---: | ---: |
| 人持续存在 | 100 | 100 |
| 人离开，地面射线出现 | 100 | 0 |
| 低障碍仍存在 | 100 | 100 |
| 人再次出现 | 100 | 100 |
| 传感器断流 | 100 | 100 |
| 恢复清除观测且人已离开 | 100 | 0 |

100/0 为发布占据栅格的 lethal/free 值。上述证据支持已有修复的特定机制；
不证明现场已部署新参数，也不涵盖所有坡面、遮挡或传感器故障。
`tests/test_stationary_obstacle_clearing_ros.py` 保留此真实插件回归，仅允许
`ROS_DOMAIN_ID=183` 且 `ROS_LOCALHOST_ONLY=1` 的隔离运行。

## 本次加入的观察项

`config/field_trial.rviz` 与 `config/field_photo.rviz` 同步调整：

- `Filtered Obstacles (marking)`：橙色，`/agt/navigation/points_obstacles`，保持开启。
- `Raw Navigation Cloud (clearing source)`：青色，`/agt/livox/points`，默认关闭；分析清除失败时开启。
- 保留 `Local Costmap`，显示实际二维合成代价，其中包含膨胀层，不标成“原始体素”。
- `Local Costmap Footprint`：红色，`/local_costmap/published_footprint`。这是当前车体轮廓，**不是独立停止区**。

点云采用 SensorData 相容 QoS，`Decay Time: 0` 避免 RViz 人为累计旧帧。
最新一帧也可能在发布停止后仍留在屏幕上；显示中有点不代表传感器仍在线，应同时检查时间戳、频率和 TF。
轮廓主题来自 Nav2 [Costmap2DROS](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/src/costmap_2d_ros.cpp#L235)。
没有添加未部署的 `Stop Zone` 或 `VoxelLayer Marked` 主题。

## 下次现场核对

先只读检查当前会话实际参数和版本：

```bash
ros2 param get /local_costmap/local_costmap plugins
ros2 param dump /local_costmap/local_costmap
ros2 node list
ros2 topic info /agt/navigation/points_obstacles --verbose
ros2 topic info /agt/livox/points --verbose
dpkg-query -W ros-humble-nav2-costmap-2d ros-humble-nav2-controller
```

若 `plugins` 仍为 `voxel_layer`，优先检查本次 Workbench 构建和启动入口，避免改一份未加载的 YAML。
记录同一时间线上的过滤云、清除云、局部代价图、TF、控制器日志、定位和 guard 状态，
用障碍持续存在、离开、重新出现、低矮障碍仍存在分别判断过滤与清除是否正确。
先用隔离回放复核断流行为；实车试验不以调低保护阈值作为通过条件。

## 本轮验证记录

- 两份 RViz YAML 可解析，标记/清除 topic 与锁定 `perception.yaml` 一致；检查了
  SensorData 相容 QoS、默认启用状态和无旧帧累计设置。
- RViz 拍照队列安全入口回归通过；真实 Nav2 清除的两个参数化用例因当前环境没有
  隔离 ROS 运行条件而跳过，未将历史结果计作本轮通过。
- 额外执行拍照入口的若干离线检查：9 项通过、2 项因缺少
  `ament_index_python` 和工作区 `/workspace/config/field_trial.yaml` 失败。
  这两项未涉及本次 RViz 修改，保留环境限制记录，没有用替身模拟 ROS 通过。
- `git diff --check` 通过。本轮未打开 ROS RViz 窗口，也未进行实车动态测试。
