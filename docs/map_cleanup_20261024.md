# 跟车线状残影复现与清理（2026-10-24）

本次三维、二维都有的长线来自已累积的点云。旧方法仍在锁定的
`agt_pcd2grid_exporter/TraversabilityGridBuilder.cpp` 中：世界坐标体素的跨关键帧
持续观测，以及至少一次超过 2 m 的观测，用于排除始终贴近车辆出现的跟随物体。
仓库原有 `FollowerSeenOnlyUpCloseIsNotAnObstacle` 测试证明该规则已存在。
当前无 PGO 路径只保存合并 PCD、trajectory.csv，没有逐帧边界或 patches；
Workbench 从单 PCD 转二维时显式关闭 temporal_filter，因此旧清理路径被绕过。

## 已接回的保存链路

1. `lio_artifact` 增加 frames.csv：时间戳、纳秒时间戳、XYZ spool 起始索引及点数。
   不改 LIO 输入，不改激光点时序，不改在线过滤保护阈值。
2. `workbench/map_cleanup.py` 严格按相同时间戳关联每一帧与位姿；检查索引连续、
   全部点覆盖、时间单调、有效四元数。缺失证据即失败，不猜合并 PCD 的扫描边界。
3. 全部 world scans 逆变换为 IMU/body patches，没有关键帧抽样或体素降采样。
   T_base_body 由本次选定 bundle 的 T_base_lidar * inverse(T_body_lidar) 合成，
   包含 base_link_height，避免套用名义默认值。
4. 调用原有 traversability 算法，新 `--cloud-output` 同时输出 static_map.pcd、
   removed_candidates.pcd 与点数报告；保留地面，包括近场或只有单次观测的地面。
   同一个持久性判定用于原生二维诊断输出，未新增第二套独立清理算法。
5. 原规则原用于稀疏 PGO 关键帧；10 Hz 原始扫描要求至少 3 次观测且跨约 1 秒
   （本包为 11 帧索引跨度），其余阈值复用原配置。2 m 规则不表示删除整个近场，
   近处静态结构只要也被远距离观测过便保留。
6. Bunker + FAST-LIO + 选定 calibration bundle 的 mapping_pipeline 保存步骤自动清理，
   导入清理后的三维 PCD，再由原来的 local_ground 生成界面二维版本和后续 BBS 输入。
   没有 bundle 的旧流程、FW-mini 或 PGO 流程不冒用 Bunker 外参；标记 NOT_RUN。
   手动导入单一融合 PCD 也不能恢复真正的时序证据。

原始 map_package/map.pcd、points.raw、rosbag 均保留。三维候选、移除点和原生
二维诊断位于独立 temporal_cleanup 目录，并记录输入、外参、配置及输出 SHA。
界面二维 local_ground 的 temporal 统计为 0，表示该步骤的输入已经完成三维时序清理；
实际清理统计在 cloud manifest 的 temporal_cleanup 中。

## 本包结果

实验：`runs/map_cleanup_20261024T1320/`，ROS_DOMAIN_ID=147、localhost-only。
完整 491.36 秒包只播放 LiDAR、IMU；未启动底盘或导航，未停止 domain 0 的传感器会话。

- 4,912 帧、10,201,128 个点，4,912 个精确位姿关联。
- 轨迹长 318.54 m，Z 跨度 0.461 m，最大相邻位移 0.112 m；仅数值合理性检查。
- 车体范围过滤 95,363 点。
- 静态证据不足的移除候选 347,600 点。
- 保留 9,758,165 点，其中额外保留近场/单次观测地面 50,621 点。
- 全量准备 patches + 清理约 16.63 秒（不含 8 分钟实时速率 LIO 回放）。
- 同参数二维 A/B：占用格由 239,571 降到 179,055；格数变化不是物理正确性证明。
- 目视对照：绕场长线显著去除，主要边界/墙线保留。起点附近仍有持久点团。

新界面名称：`bunker 农科院 建图测试1 去残影候选`。
cloud ID：`20261024T124930Z-714128a2`；grid ID：`20261024T124938Z-f71341f4`。
状态 DRAFT，未切换 active map，未生成这份候选的 BBS 资产，未进行实车导航验收。

## 验证与边界

- C++ 原有/扩展 18 项测试通过；跟车用例同时检查 3D 去除、静态柱保留、地面保留。
- Python 9 项通过：严格帧关联/外参方向、补丁幂等与保留用户编辑、二维与资产桥接。
- 活跃 install 的 bringup/exporter 已重建；完整包通过新增帧索引导出链路。
- 最终正式 install 的 exporter 再处理全部 4,912 帧：静态 PCD、移除 PCD、点数报告及原生栅格与候选运行逐字节一致。
- 追加零移除点用例发现 PCL 拒绝写空点云；已处理合法空 PCD，失败 XML 单独保留。
- 补丁以 map_cleanup_manifest.json 保存精确基线和哈希；原来的两个 bringup 集成修改
  原样纳入可复现补丁，没有覆盖用户其他修改。
- dependency audit 无 dirty/duplicate；V3 的 SHA 不匹配是此前已存在的状态，未重置。
- 初次 isolated CTest 误加载旧 underlay 动态库，source 候选 overlay 后及正式 install
  构建后的 CTest 均通过；保留兼容的旧 build API。

这不是语义人体检测。长时间站立、被重复观测的人/临时物体可能保留；只被近距离
看到的真实静态物体也可能进入移除候选。不能据占用减少就认定可通行或几何完全正确。
下一步若还需清起点点团，应结合原始逐帧画面确认 ROI，再做有记录的编辑，不能凭外观
自动挖掉可能的真实障碍。

在线自体滤波本包另有 3,346 帧触发 BYPASS_GUARD，保护逻辑选择传回原扫描。
本次没有放宽其 30% 保护阈值；最终地图阶段仍按实际外参补做车体范围清理。
此计数不证明线状残影全部来自人体或全部来自车体。

## 云端可查看的对照图与代码

[清理前后图片、验证摘要与三个仓库分支](evidence/mapping_cleanup_20261024/README.md)。
