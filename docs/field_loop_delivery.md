# 双底盘闭环集成交付

> **2026-10-25 后续更新：** 本文是 10 月 23 日的历史验收快照；其中“KISS 不开放实时默认切换”仅描述当时状态，已被 `feature/agri-sc-kiss-20261025` 的工作台集成更新取代。当前 Qt 默认 SC，尝试从同一建图产物生成 SC 数据库和 BBS 候选索引；二维像素编辑保留同源 3D 定位资产。真实 bag/硬件/定位精度仍未验收。

2026-10-23；Ubuntu 22.04.5 / ROS 2 Humble；同一工作区 `agt_field_test_ws`、集成仓库 `main`。

本轮开始远端/本地 HEAD：`a1a9d7c26d68629cc59b73bcd42d681fb502cefd`，工作树干净；`61b2f930…` 为历史基准。已保留 FW-mini 原候选文件，逐字节与起始提交比对相等。未重置依赖仓库、未新建长期分支。最终唯一候选 SHA 为本次发布提交（`git rev-parse HEAD`）；发布后 `scripts/write_delivery_manifest.py` 生成绑定该 SHA 的工作区报告。

## 已接入的实际路径

| 范围 | 软件状态 | 本轮内容 / 证据 |
|---|---|---|
| P0 双底盘 profile | PASS | 同一 profile 注入 RPP、smoother、guard、driver；Bunker 0.15 m/s 不再被旧 0.40 巡航覆盖。旧 profile 失效不会阻断地图工具；可另存新版并选用，原文件和速度限制保留，[云端版本迁移记录](evidence/field_loop_20261023/profile_migration.json)。独立受保护话题，Bunker 非全向，FW-mini 6/8 轴约束，反馈/控制权、deadman、命令超时清除意图。 |
| FW-mini 控制与 TF | PASS（软件） | [9 项控制回归](evidence/field_loop_20261023/fwmini_control_final.log)、[4 项 TF 回归](evidence/field_loop_20261023/fwmini_tf_final.log)、[实际 CtrlCmd 消息序列 gzip](evidence/field_loop_20261023/ctrl_cmd_sequences.jsonl.gz)。无 CAN。 |
| P1 Bunker 标定运行接入 | PASS（候选接线） | 独立 mount/internal/RTK/time 包；RSP 真正应用 mount；两种 LIO 各自冻结内部参数，保留原始正负号。导航配置保留 odom/body，建图保留其自身世界帧。无测量的杆臂不发布。 |
| P1 时间数据链 | PASS（合成 DDS） | 原始→校正 IMU/CustomMsg→算法；Livox header/timebase 同移，逐点 offset 不变。全局查询与 LIO 共用校正后的输入。实际 PTP 精度未测。 |
| P2 无 PGO 导出 | PASS（合成 DDS） | FAST-LIO2 + 独立 world scan/trajectory exporter；真实 launch 图无 PGO，无底盘命令发布；导出不读 PGO 目录/服务。 |
| P2 全量 PCD / 2D / 路线版本 | PASS（软件） | PCL 全分辨率 ROI 删除/裁剪/体素，参数化 PGM；20,000 点测试包含预览以外部分、输入 SHA 不变。DRAFT 可编辑；地图/路线绑定 ID、SHA、frame。 |
| P3 KISS + small_gicp | PASS（可选构建与合成接口） | 固定 KISS SHA、额外构建锁，ROBIN/pybind 构建问题通过只构建 Python target 与固定依赖解决；未修改源仓库。BBS/KISS 使用相同 PCD 和同一 native small_gicp。[锁定构建](evidence/field_loop_20261023/kiss_locked_build.log)、[合成对照 JSON](evidence/field_loop_20261023/synthetic_registration_locked.json)。 |
| P3 真实定位基准 / Tracker A/B | NOT_RUN | 提供同工作区离线 benchmark 与顺序 tracker on/off runner、候选/manager/TF 日志。无真实 query bag/真值；KISS 不开放实时默认切换。粗检索为明确的子图/径向描述符排序，召回率未验收。 |
| P4 INS 接口 / GPS 先验 | PASS（构建与策略测试） | `agt_asensing_driver` 固定 SHA，Humble 构建与上游解析/契约测试；只消费三个正式话题。可选已验证 ENU/杆臂先验 gate 不发 TF，退化移除约束，跳变不直接改 map→odom。 |
| P5 网页操作 / Action | PASS（软件） | 会话启动/停止、受监督租约、地图绑定路线和相对目标；真实 Humble NavigateToPose/FollowWaypoints/GetState 协议以模拟服务端验证，物理门禁显式 mock，**不是 Nav2 规划或运动验收**。 |
| P6 摆动诊断工具 | PASS（合成 bag） | 同时间轴指令/LIO 导数、wheel-LIO、修正量、延迟、FFT、偏差与时间缺口；合成测试回收 0.6 Hz、0.1 m 修正步长和约 0.04 m 横向偏差。[结果](evidence/field_loop_20261023/synthetic_diagnostics.json)。 |
| P7 统计与报告 | PASS（软件） | 正式 12/25/35 cm 与旧工程 12/15/20 cm 并列，5 次完整物理证据才判定；AP50 实现需图像/GT/模型证据。缺数据保持 NOT_RUN。 |

## 测试证据

- [58 包目标构建及测试入口日志](evidence/field_loop_20261023/full_build_humble.log)；135 项 colcon 测试 0 error / 0 failure。随后最新 Python/launch 的增量构建与对应集成测试亦执行。
- [22 项 Python 单元/补丁重放测试](evidence/field_loop_20261023/unit_final.log)。包括全新锁定检出应用两次、保留用户修改、厂家 encoder/AliveCounter/BCC 不变。
- [Humble 原生/DDS/launch/录包集成](evidence/field_loop_20261023/ros_field_loop_final.log)：实际 RSP、Bunker 反馈/hold 门禁、Bunker 定位查询和 LIO 校正话题、无 PGO exporter/图、时间移位、raw bag 正常关闭。
- [Humble RPC、NavigateToPose/FollowWaypoints/GetState 与合成诊断](evidence/field_loop_20261023/final_runtime_recheck.log)。Action 服务端与物理门禁是测试替身，不代表 Nav2 轨迹通过。
- [实际会话进程生命周期](evidence/field_loop_20261023/session_lifecycle_final.log)：无 CAN/传感器，独立 job/session 身份、启动、RPC、停止、会话指针清除。
- [浏览器编辑并保存 2 航点路线](evidence/field_loop_20261023/browser.json)，[路线界面](evidence/field_loop_20261023/workbench_routes.png)、[诊断界面](evidence/field_loop_20261023/workbench_diagnostics.png)。临时合成地图/路线已清理。
- [环境版本](evidence/field_loop_20261023/environment.txt)、[依赖审计](evidence/field_loop_20261023/dependency_audit_final.json)：17 个主依赖、79 个源码包名，重复 0、SHA 偏差 0、非预期 dirty 0。KISS 源目录有明确预期 COLCON_IGNORE，不参与基础 build。

首次测试失败日志一并保留：启动失败后测试残留发布者污染域、异步 DDS discovery、Bunker FAST 导航模板帧名以及布尔 launch 参数。已分别修复测试清理/就绪等待和实际配置代码，再做对应真实 Humble 复测。`bunker_localization_debug*.log` 为修复前证据；不能单看失败历史认定最终结果，也不能删去失败证据冒充首次即通过。

## 仍需现场执行 / 限制

| 门禁 | 状态 | 所需证据 |
|---|---|---|
| Bunker mount/internal/时间/旋转中心新标定 | NOT_RUN | 静止、旋转、直行、八字重复测量、离线重放；当前 CANDIDATE。在线估计收敛日志未实现为正式标定输出，不能据开关声称完成。 |
| FW-mini 原候选的物理验证升级 | NOT_RUN | 序列号、旋转中心、安装方向、重复性、几何、定位和安全报告；保持 CALIBRATION_CANDIDATE。 |
| 两车 CAN、架轮零速、制动、急停、遥控接管 | NOT_RUN | 先反馈/档位核对与架轮，再落地低速监督；零速消息不代表真实制动。 |
| 真实 raw bag 建图与不同外参重建 | NOT_RUN | 当前没有实车 bag，合成 exporter / recorder 不替代整段 LIO 重建。 |
| 地图与路线版本的现场审核 | NOT_RUN | 本轮没有实测地图/路线；不能把合成浏览器 fixture 作为交付场地图。 |
| BBS / KISS 真实成功率、误接受、歧义、恢复、CPU | NOT_RUN | 同一真实 query bag/外参/地图与独立真值；合成单例没有统计代表性。 |
| Tracker 校正开/关真实 A/B、TF 连续性 | NOT_RUN | 同 bag 两次回放、manager 接受证据、map→odom 变化。 |
| GPS 硬件质量 / 地图 ENU / 杆臂 | NOT_RUN | 固定解/有效位/标准差/时间新鲜度、独立地图原点航向与杆臂实测。 |
| GPS 因子图联合优化 | NOT_RUN | 本轮没有实现第二阶段优化；不启用 PGO。 |
| FW-mini 横移自主导航 | BLOCKED | 8 档当前仅受监督 jog；现有 RPP 路径只允许 6 档，需要独立横移控制器验收。 |
| Bunker 摆动分类及单变量参数 A/B | NOT_RUN | 真实分阶段 motion_diagnostic bag，工具输出只是分类线索。 |
| Nav2 真实规划与底盘跟踪 | NOT_RUN | 当前 Action 测试只有协议与门禁；实车需正式门禁。 |
| 5/7/10 m ×5、逐次误差与平均值 | NOT_RUN | 独立测量真值和证据 SHA，不能填合成结果。 |
| 多点拍照与花穗/果实 mAP50 | NOT_RUN | 相机 waypoint task executor 尚未配置；真实图片/GT/模型和逐点任务证据缺失。 |
| 正式论文原文对照 | NOT_RUN | 当前未找到上传原件；阈值依据本次用户文本，待上传后复核方法定义。 |

正式导航必须有 ACCEPTED 地图、VERIFIED 外参、绑定同一 profile/map/calibration 的 PHYSICAL_NAVIGATION 报告，以及实时定位、反馈、控制权与 Nav2 ACTIVE。本轮未启用自主导航。

逐页操作、报告 schema、架轮/落地/精度分阶段步骤见 [操作手册](workbench_operator_manual.md)。现场部署后先重复构建和无运动检查，再按手册逐项执行并保存新的不可覆盖结果。
