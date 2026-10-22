# AGT Field Test Workbench 交付与现场验收记录

记录日期：2026-10-23（用户本地时间 Asia/Shanghai）。现场清单日期：周六 2026-10-24。

## 仓库审计证据

完整机器可读结果：[dependency_audit_snapshot.json](dependency_audit_snapshot.json)。审计脚本读取每个实际 HEAD、工作树和 `package.xml`；13 个固定依赖中检出 73 个 ROS 包，重复包名 0，SHA 偏差 0。建图仓库的两处集成补丁被标记为 `EXPECTED`，未改其 HEAD。原 `/workspace/agt-lio-pgo-mapping` 位于 `work` 分支、HEAD `11361fca3bcb1e664d45760944621330c8eb0eee`、工作树干净；新工作区独立检出远端 `main` 固定 SHA `62e80002cf2db12fce8332688dc63f4a94b87f12`，以满足本任务指定的 FAST-LIO2 + PGO 基线。旧工作区未修改。

| 来源 | 固定 HEAD | 关键发现 |
| --- | --- | --- |
| `agt_navigation_v3` | `b3a60487dff973c6326aca2ca2e9db1e78daeaaa` | Batch/FAST 两个适配器均指向 `/agt/odometry/local`；LocalizationManager 唯一发布 map→odom；tracker 的 `apply_correction: true` 有真实 pose 发布和管理器接收路径。旧 hardware launch 仍引用不存在的 `bunker_base` 包名。 |
| `agt-lio-pgo-mapping` | `62e80002cf2db12fce8332688dc63f4a94b87f12` | 已有 FAST-LIO2 + PGO bag/live CLI、raw bag、map package 校验与 PCD→PGM exporter；集成补丁允许按版本传入 `r_il/t_il`。 |
| `agt_pointcloud_pipeline` | `2def1b76b7526b88972a3bc747b47dd7badcc765` | 点云诊断与过滤源独立；本工作区没有把过滤接入原始 LIO 输入。 |
| `fwmini_navigation_lab` | `34ac35650452027f6f8efde0e3c4ace6ca526a9f` | `main` 可明确检出；默认 HEAD 引用不能直接解析。厂家 `yhs_can_control`/`yhs_can_interfaces` 许可均为 TODO；Humble 构建与真实 CAN/运动未验收。 |
| `agt_bunker_base` | `cb6c3655ad7d85d8a1e847342eefa20f90d102ed` | 实际 ROS 包名为 `agt_bunker_base`，启动参数默认 `/mux/cmd_vel`、`/wheel/odom`，`publish_odom_tf=false`。 |
| `agt_chassis_description` | `75b838ce9af0d8da7c403d6683b34f4e4268f687` | 实际包名 `tracked_chassis_description`，提供 Bunker URDF/MID360 安装基线。 |

依赖冲突：V3 的 Livox SHA `4a1def9` 与建图的 `13eb05e` 不同；V3 FAST-LIO2 `f516daa` 与建图 fork `7f664a6` 不同；Batch-LIO `fe4a9cb` 与建图 fork `7ceb276` 不同。新 `.repos` 只选 Livox `13eb05e`、FAST-LIO2/PGO `7f664a6`、Batch-LIO `fe4a9cb`。V3 的 FAST-LIO2 导航后端与建图 fork 的组合尚无运行证据，默认导航使用 Batch-LIO；若不兼容，须另设隔离 overlay。绝不同时启动两个局部里程计适配器或两个 `odom→base_link` 广播器。

## 验证状态

| 项目 | 状态 | 证据或后续命令 |
| --- | --- | --- |
| 固定 SHA / 重复包名 / 源树审计 | PASS | `python3 scripts/audit.py`；快照如上。 |
| Python 后端地图版本、录包完整性、进程组取消、binary PCD 解析 | PASS | `PYTHONPATH=. python3 -m unittest discover -s tests -v`，4 项通过；[测试日志](test_results_cloud.txt)。 |
| 网页资源和 HTTP API smoke | PASS | `/`, `/app.js`, `/style.css`, `/api/status`, `/api/jobs`, `/api/maps`, `/api/templates` 均 HTTP 200；无 ROS 建图请求 HTTP 400，明确报错；[HTTP 结果](http_smoke_cloud.json)。 |
| JS 语法和 Python 编译 | PASS | `node --check workbench/static/app.js`；`python3 -m compileall -q ...`。 |
| Ubuntu 22.04 / ROS 2 Humble 全量构建、colcon test | BLOCKED | 当前主机 Debian 13，`/opt/ros/humble/setup.bash` 不存在；[build_humble_cloud.log](build_humble_cloud.log) 记录退出码 2。需在工控机运行 `bash scripts/build_humble.sh`，保存日志。 |
| FAST-LIO2 + PGO 真实 bag 回放、自动 PCD→PGM | NOT_RUN | 无 rosbag、ROS 运行时和传感器；网页 Mapping 指定真实 bag 后执行。 |
| Bunker / FW-mini CAN、里程计、指令超时、急停和低速运动 | NOT_RUN | 当前无 CAN/底盘；逐车执行现场清单。 |
| 持续校正实际 TF 更新与关闭校正 A/B | NOT_RUN | 代码路径存在；须用同一 bag 分别 `enable_map_tracking:=true|false`，检查 JSONL、metrics 和 TF 唯一性。 |
| Nav2/costmap/正式自主路线 | NOT_RUN | 未有地图、定位质量、实机运动验收；网页运动操作未开放。 |

## 周六现场操作清单（两车分别记录）

每一步在 `reports/field_20261024_<robot>.md` 写时间、操作者、bag/地图/标定 SHA、日志路径和 `PASS/WARN/BLOCKED/NOT_RUN`。没有测量结果的项目保持 `NOT_RUN`。

1. 工控机 `git clone`、`bash scripts/init_workspace.sh`、`bash scripts/build_humble.sh`；保存 `reports/dependency_audit.json`、`reports/colcon_test_result.txt` 和完整构建日志。若厂家包构建失败，先记录错误，不启动运动。
2. 检查 MID360 IP/主机网卡、静止 IMU 单位/重力/频率/时间戳；用 Calibration 页或 Recorder 的 `calibration` 模板录包。记录传感器序列号和 Bunker/FW-mini 各自安装外参。
3. 验证实体急停、遥控接管、厂家控制模式、CAN 波特率和反馈新鲜度。Bunker 检查 `/wheel/odom`；FW-mini 检查 `/chassis_info_fb`、`/wheel/odom`，确认其预览几何已替换为实测 YAML。逐车核对指令超时和停车。未经这些 PASS，不调用 `/fwmini/enable_motion` 或发布速度。
4. 仅在受控区域、低速监督下测试 0.05 m/s 直行/停车；核对实际制动距离、轮速和 `odom→base` 发布者。先停止前一底盘实例再切换；推荐用 `scripts/run_field_test.sh` 的独占锁。
5. Recorder 选择 `mapping`，或 Mapping 页选择实机建图（后者自动录一次 raw bag，不叠加第二路录制）。检查 `metadata.yaml`、消息数量和缺失话题。随后用同一 raw bag 离线重跑 FAST-LIO2 + PGO。
6. 以新标定版本重建地图。检查 `map_package` 的 PCD/轨迹/校验清单和 `map_grid/map.pgm`、`map.yaml`；在 Map Studio 预览 3D/2D、编辑并保存新版本。保留源 PCD 和 bag 不变；未验收先标 `DRAFT` 或 `PROVISIONAL`。
7. 在地图与外参身份一致、TF 连续后启动定位；记录全局粗定位、small_gicp、tracker 候选/拒绝、manager 状态和 `map→odom` JSONL。用同一 bag 关闭 tracking 做对照，不以单次收敛判定成功。
8. 最后验证 Nav2 map/costmap、footprint、局部规划与目标区域可达；仅在前面门禁 PASS 后执行监督低速路线。分别记录两车结果，不合并为“软件通过”。

## 未解决问题与运动影响

| 问题 | 影响实机运动 | 处理条件 |
| --- | --- | --- |
| 两车 Humble 构建、CAN 和反馈尚未现场执行 | 是 | 工控机 build 和逐车反馈/急停/遥控/超时测试 PASS。 |
| FW-mini 厂家许可未知、控制模式与实测几何缺失 | 是 | 确认使用/再分发许可；完成厂家模式和几何验收。新仓库未复制厂家源码。 |
| FW-mini MID360 安装 TF 未测量，定位 launch 明确阻断 | 是，影响自主导航 | 完成独立安装 profile、URDF 和数据验证。 |
| Bunker V3 旧硬件 launch 包名不符 | 新入口已绕开；旧入口受影响 | 统一入口实测启动；无需改旧 `main`。 |
| 标定时间偏差和车体安装外参尚未接入算法/URDF | 是，影响正式地图/定位 | 完成转换与实测一致性检查。仅 `r_il/t_il` 目前能选版本用于建图。 |
| 地图身份门控和校正 A/B、逐候选 manager 接受证据尚未验证 | 是，影响自主导航 | 真实 bag/现场日志审核，并确认唯一 map→odom 广播源。 |
| Map Studio 状态 `ACCEPTED` 当前为人工填写理由，无场地语义检查自动化 | 是，影响正式自主任务 | 现场地图、footprint、costmap 和路线审核。 |

网页的 Navigation 区域故意没有运动按钮；当前软件验证不构成实车验收。
