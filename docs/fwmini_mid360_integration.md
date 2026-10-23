# FW-mini MID360 外参导入与运行时集成（2026-10-23）

## 来源核对

云端没有原始 `~/ros2_ws/experiments/fwmini_rviz_nav_candidate_20261021/fwmini_profile_candidate.yaml`，也没有用户工控机的 `~/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/`。本次从 `Aldoubt/fwmini_navigation_lab` 恢复了 Git 原文，未改变依赖仓库的工作树或锁定提交：

- `94a5c5bd679c9980f30d16b475a88a8e06ef7334`，`feature/fwmini-field-profile-usable-20261022`，2026-10-22 19:04:51 +08:00。其描述为现场试验可用，精度验证仍待完成。
- `aad4b6eb955ebb4de6444d40dc7d9e1b14f879a8`，`feature/fwmini-mid360-route-workbench-20261022`，2026-10-22 17:58:55 +08:00。其描述明确为未现场验证，并指出拟合旋转中心与早期车体中心描述存在差异。

两份数值一致，验证措辞不同；[原文快照和逐文件 SHA-256](evidence/fwmini_20261022/provenance.json)一并保留。原记录没有完成实车建图、运动或重定位验收，不能据软件检查升级为实车通过。

导入文件：`agt_field_test_bringup/config/fwmini_mid360_candidate.yaml`。初始化脚本将其复制至工作区 `calibration/fwmini/fwmini_mid360_candidate.yaml`，不覆盖现场已有文件。状态保持 `CALIBRATION_CANDIDATE`。

| 项目 | 候选值与语义 |
| --- | --- |
| 适用底盘 | FW-mini，厂家尺寸 495 × 360 × 320 mm，轴距 350 mm，轮距 250 mm；独立精测待完成 |
| 安装变换 | `T_base_link_livox_frame`：`p_base = R * p_livox + t` |
| xyz / m | `[0.146725, 0.002608, 0.377040]` |
| URDF 固定轴 RPY / rad | `[0, 0.2152, 0]`；正 pitch 使雷达 x 轴朝前下方 |
| 高度来源 | CAD 雷达中心距地 0.44704 m，减去 base_link 距地 0.070 m |
| XY 来源 | FAST-LIO 旋转测试拟合，假设运动旋转中心等于 base_link 原点 |
| pitch 来源 | 静态 LIO 姿态中值；装配方向和重复性仍需独立验证 |

147 mm 的拟合 x 不能直接解释为已测得的车体几何中心。现场需复核旋转中心、轮胎滑移影响和当前安装身份。

## 实际 TF 与算法接入

```text
base_footprint ──[z=0.070]──> base_link
                              └──[候选 xyz/rpy]──> lidar_link
                                                    ├──[单位变换]──> livox_frame
                                                    └──[内部 T_imu_lidar 的逆]──> imu_link
```

四条实体固定边由一个 `robot_state_publisher` 发布。`lidar_link` 与 `livox_frame` 表示同一雷达坐标，安装变换只应用一次。旧 FW-mini 描述 launch 不再同时启动。轮里程计适配器只发布消息，关闭其 TF；定位 LIO 适配器负责 `odom → base_footprint`，LocalizationManager 负责 `map → odom`。建图使用原始世界帧，经现有适配器将 LIO body 姿态转换为车体姿态，发布至 `/mapping/fwmini/base_odometry` 及世界帧到 `base_footprint`。

内部 LiDAR–IMU 变换与车体安装外参分别保存：候选沿用锁定 FAST-LIO 的 `r_il=I, t_il=[-0.011,-0.02329,0.04412]`，语义为 `p_imu=R_imu_lidar*p_lidar+t_imu_lidar`。因此默认 `lidar_link → imu_link` 平移为 `[0.011,0.02329,-0.04412]`。此内部参数仍是假设适用于当前装配。锁定 Batch-LIO 配置原有平移符号与 FAST 不同；FW-mini 定位启动生成专用 FAST/Batch 配置，统一采用所选 profile 的内部变换，关闭在线外参估计；未改动其他底盘的配置。

Humble C++ TF guard 用 DDS publisher GID 检查固定边的数值、方向和唯一发布权，并监视 `base_footprint`、`odom` 的重复父节点/发布者。先通过真实 `/tf_static` 检查，再启动定位或建图适配器；运行中冲突触发整个 launch 停止。RSP 退出也触发停止。报告 `SOFTWARE_TF_PASS` 明确标记 `physical_calibration_verified=false`。

建图任务冻结所选安装 profile 和内部参数，在独立 ROS domain 89 启动 TF，记录摘要及检查报告；原有 bag 播放仅回放传感器话题，不回放 bag 中的 TF。录包入口对当前 ROS 图执行实际 TF 检查，不另起第二个 RSP。存在旧描述节点时应先停止旧节点再启动该入口。

## 允许模式

| 模式 | 候选外参条件 |
| --- | --- |
| `sensor_test` | 配置合法、实际 TF 通过 |
| `calibration_recording` | 配置合法、实际 TF 通过；Recorder 保存所选版本 |
| `sensor_mapping` / `offline_mapping` | 同上；输出保持 DRAFT，需审核 |
| `supervised_localization` | 同上，显式 `supervised:=true`，有效全局地图 |
| 正式自主导航 | 候选禁止；VERIFIED 安装外参加定位、几何、运动安全验收仍是必要条件。当前 FW-mini 尚无独立验收的 Nav2 profile，正式入口保持关闭 |

在工作区根目录执行传感器检查（默认不启底盘）：

```bash
bash src/agt_field_test_workbench/scripts/run_field_test.sh robot:=fwmini fwmini_mode:=sensor_test
```

受监督定位观察示例：

```bash
bash src/agt_field_test_workbench/scripts/run_field_test.sh \
  robot:=fwmini enable_chassis:=false enable_localization:=true \
  fwmini_mode:=supervised_localization supervised:=true \
  global_map:=/absolute/path/global_map.pcd \
  fwmini_mount_file:=/absolute/path/calibration/fwmini/fwmini_mid360_candidate.yaml
```

默认候选无需手动传 `fwmini_mount_file`。离线无雷达测试可设 `enable_mid360:=false`。网页选择安装 profile 后，新的建图/录包任务使用该版本；运行中的节点不会被网页热切换。底盘运动仍需独立的实测几何文件、急停、控制模式和运动验收。

## Calibration 页面与现场升级

页面显示来源提交、数值、状态、假设、待验证项和 calibration SHA-256。现场完成验证后，将真实报告保存至工作区 `reports/` 或 `runs/`，填写传感器序列号和报告路径，点击生成 VERIFIED 版本。模板见 [fwmini_mount_validation.example.json](../templates/reports/fwmini_mount_validation.example.json)。

报告必须具有 `verification_scope=PHYSICAL_MOUNT`，绑定相同的 robot、sensor_id 和页面 calibration_sha256；五项安装检查均为 PASS，并填写操作者、观测日期和实际观测。生成新 YAML，保留原候选，通过报告 SHA-256 防止升级后修改证据。纯软件 TF 报告不能用于升级。VERIFIED 仅表示安装检查等级，定位和运动安全仍保持未验证，不能自动开放导航。

## 本轮测试结果

2026-10-23，Ubuntu 22.04 / ROS 2 Humble Docker，源码和安装工作区真实运行：

- `colcon build --packages-select agt_field_test_bringup`：PASS。
- `python3 -m unittest discover -s tests -p 'test_*.py' -v`：11/11 PASS（含 7 个新增外参/API 用例）。
- `node --check workbench/static/app.js`：PASS。
- 实际 Humble 建图任务 TF 编排烟测：PASS；验证快照、真实 TF ready 和清理，不含传感器建图。依赖审计无 SHA 漂移、重复包或未预期修改。
- `ROS_DOMAIN_ID=214 bash scripts/ros_exec.sh python3 tests/humble_fwmini_tf.py`：4/4 PASS。实际 RSP、TF2、DDS GID 冲突检测、缺边拒绝、候选导航/无监督定位拒绝，以及受监督定位 launch 和合成 LIO 数据到车体姿态转换均覆盖。

[机器结果摘要](evidence/fwmini_humble_20261023/result.json)。后一个命令需从仓库目录执行，或按工作区位置补齐 `src/agt_field_test_workbench/` 前缀。测试脚本在工作区 `reports/fwmini_tf_humble/` 保存机器结果和 launch 日志。合成点云/里程计只用于验证软件链路，没有雷达、CAN、底盘或物理标定通过的含义。

## 仍需实车完成

传感器序列号与安装一致性；独立旋转中心到 base_link 测量；pitch/高度/方向与点云检查；直行和原地旋转的重复残差；内部 LiDAR–IMU 方向、单位和时间同步；真实建图与同包重定位对照；实际 footprint、定位质量、急停/遥控接管/CAN/超时/低速运动验收。原始 10 月 21 日拟合数据仍需从工控机补回，新增证据应保留来源和摘要后再决定是否调整数值。
