# Field automation 20261024 delivery

交付整理日期（Asia/Shanghai）：2026-10-25；分支名称保留原任务日期。

目标分支：`feature/field-automation-20261024`。开发前 `git ls-remote`、clone、fetch 均确认远端为
`82cb85f8bbefd86d8bb7b9c1208a065fa2c5e9fd`，不是把历史 SHA 当成当前状态。原云端目录是另一个
mapping 仓库，保持其工作区不变；本任务在单独检出的指定功能分支上增量开发。没有合并 main。

## 状态与边界

| 范围 | IMPLEMENTED | AUTOMATED_TESTED | HARDWARE_VERIFIED |
| --- | --- | --- | --- |
| A 云台与采集 | 已实现，沿用 C1 AcquireView 与现有 mission_runtime | Python 结果校验、真实 Humble Action + 软件设备夹具 | NOT_RUN |
| B 实时三视图 | 已实现，二维栅格、三维 XY 俯视、XZ 侧视 | Qt、队列、TF、6500 点真实 ROS 消息链路 | NOT_RUN |
| C 独立生命周期 | 已实现，沿用 Jobs/Sessions，定位与 Nav2 独立进程 | Bunker/FW-mini 软件启停、所有权转交、失败/孤儿/停止失败测试 | NOT_RUN |
| D 地图差异审核 | 已实现，只生成独立 DRAFT，人工审核后仍不自动激活 | 对齐、哈希、分类统计、图像、审核与资产失效保护 | NOT_RUN |

没有宣称完成实时动态物体过滤、点云自动配准、自动融合或自动更新正式地图。
不同来源 PCD 必须具有共同坐标基准，或由操作员明确提供已核对的外部配准/测量依据及对应源 SHA；
输入依据是可追溯的人工声明，不是本工具计算出的配准证明。分辨率、方向或整数栅格偏移不满足要求时拒绝比较。

## 针对性审查发现与修复

1. 原 RGB PNG 使用字面转义字符串，签名和 scanline 错误；修复并核验可解码图像。补齐 GUI 的 QPixmap 导入。
2. 原 GUI 只有单个三维视图；新增实时三视图，保留现有地图编辑器、录包和路线操作。
3. 原 odom 数值直接用于显示且点云没有坐标身份；实时点云按消息时间查 TF 后统一到 map。
   机器人只采用新鲜的 map→base_link TF。无效 TF/过期帧不覆盖有效快照，旧机器人标记变黄。
4. 原每次栅格刷新会重置缩放和平移；改为保留每幅视图的交互状态，三维缩放基准也不会随新点云范围跳变。
5. 输入各保留一个最新 cloud/grid，输出每流一个最新快照；Qt 定时取帧。解码、TF、点云抽样及投影准备均在后台线程。
6. 原独立进程共用 field_test.lock，第二个组件启动会被拒；改为 sensor/chassis/description/localization/control 资源锁。
7. 原 GUI 切模式先停止当前会话，会误停传感器；改由 Sessions 检查依赖、转交守护所有权。
   定位 field_session 与 navigation_component 分开，停止 Nav2 保留定位、底盘和传感器。
8. 原实时建图仍查旧 field_session 类型；改为复用运行中的 sensor_stack。
9. 原常驻组件正常码退出会显示 PASS；现为 FAILED。停止超时保留 STOP_FAILED 所有权，重复启动被拒。
   本地监督进程崩溃后仍存活的子进程组显示 ORPHANED，明确停止后才能重新转交资源。
   未经接管的历史进程也显示 ORPHANED，拒绝盲目重启/向未知 PID 发信号；检查 Linux 进程起始身份以免误认复用 PID。
10. 原相机导入失败可能没有结果文件；现写失败结果和审计记录。校验 action status、error code、实际角度、
    新图像时间戳、文件边界、PNG/JPEG 签名、尺寸与 SHA；保存拍摄时刻 TF 位姿及 pose_valid。
11. 真实 C1 的 save_image=false 仍等待新图像，只是不保存文件；保持该接口语义。失败不停止其他组件。
12. 原地图差异硬编码像素阈值、旋转原点偏移计算错误且允许四分之一格近似；改用 YAML 的 negate/阈值、
    在基准栅格坐标中计算偏移并严格拒绝未对齐图。
13. 原 diff_report 路径只放返回值、不落 manifest；现持久化报告/图像及各自 SHA，审核前重新校验身份。
    编辑几何后清除旧 source_pcd 关联，禁用旧定位资产；禁行区作为独立的保守规划约束保留原 PCD。
    比较后基准/候选的身份或审核状态变化也会阻止接受；没有绑定报告的旧 diff 草稿必须重新比较。
14. 原 Studio colcon test 默认实际运行 0 项；补 pytest 测试声明，最终实际运行 32 项。
15. 原 HTTP 队列会让停止按钮排在长时间地图处理后；增加独立停止请求 I/O 线程，仍只调用同一后端和控制链，
    没有新增任务管理器或 ROS 运动客户端。测试确认资产请求未结束时停止已到达后端。

## 修改文件和真实接口

- 后端：`workbench/devices.py`, `session.py`, `runtime.py`, `maps.py`, `routes.py`, `server.py`。
- ROS/进程入口：`scripts/camera_capture.py`, `ros_bridge.py`, `ros_exec.sh`, `run_field_test.sh`，
  `agt_field_test_bringup/launch/field_test.launch.py` 及包依赖声明。
- GUI：`agt_robot_studio/agt_robot_studio/{ui,ros_worker,workbench_client,visualization,tri_views}.py`，
  Studio package.xml/setup.py、测试与 CI。
- 依赖锁：`dependencies/field_test.repos` 增加真实 C1 仓库
  `Aldoubt/Autolabor-C1-ROS2@10d82179dc34ae035b382e9b465e4b8f5d55c0a8`。
  已从该源码逐字段核对 `src/camera_gimbal_interfaces/action/AcquireView.action`、
  `CapabilityHealth.msg`、`PantiltAngleInfo.msg` 和 launch 参数。

复用 `/camera_gimbal/acquire_view`，Goal 包含 heading/roll/pitch/capture_name/tolerance/timeout/
stable_samples/settle_time/image_timeout/save_image/tag；Result 使用 success/error_code/message/
image_path/reached_stamp/image_stamp/actual_heading/actual_roll/actual_pitch；Feedback 使用真实的
state/stage/current_heading/current_roll/current_pitch/max_error/stable_count。
健康来自 `/camera_gimbal/health`，实际角度来自 `/pantilt_camera_serial0/pantilt_angle_info`。
Bridge 仅检查能力图和健康，路线交给现有 `/agt/mission/execute`，不创建第二套巡检执行器。

新/对齐的 HTTP 接口：

- `GET /api/components`：组件进程状态、反馈门控、启动等待/超时和错误原因。
- `POST /api/components/{sensors,chassis,guard,localization,navigation}/stop`；
  `POST /api/components/guard/start`。通用 jobs/stop 也经组件依赖检查。
- 保留 `/api/session/start`，task=navigation 复用匹配的定位与底盘；不匹配时要求显式停止旧定位。
- 保留相机 start/stop/move/capture/health 接口；`GET /api/devices/captures/export` 导出采集图片、
  JSON/JSONL/CSV/YAML 记录。现有 mission_runtime 的 record_root 指向 `data/captures/missions`。
- `POST /api/maps/diff` 输入 base_id/candidate_id，可附 alignment；
  `GET /api/maps/{id}/diff` 重开已保存审核图和报告；`POST /api/maps/{id}/status` 提交状态和人工理由。
  GUI 禁止审核尚未显示或与当前选择不一致的报告。
- Launch 增加 `external_localization`，与 enable_localization 互斥。独立底盘不启动传感器/RSP/第二个 CAN。
  单一 TF watcher 随 description/sensor owner 运行，各组件保留 TF preflight。

## 实际验证

日志在 `docs/evidence/field_automation_20261024/`。Python 3.12 主机虚拟环境和 Ubuntu 22.04 / ROS 2 Humble
容器分别执行；软件夹具均未连接硬件。以下命令中的 `$REPO` 为仓库，`$C1` 为锁定的 C1 源码，
`$WS` 为测试 colcon 工作区。依赖补丁回归另外使用独立、锁定 SHA 的依赖检出，不修改原云端 mapping 仓库。

```bash
PYTHONPATH=.:agt_field_test_bringup:agt_dynamic_map_cleaner:agt_robot_studio \
  QT_QPA_PLATFORM=offscreen python -m pytest -q tests agt_robot_studio/test --disable-warnings --tb=short
python -m compileall -q workbench scripts agt_robot_studio agt_field_test_bringup agt_dynamic_map_cleaner
bash -n scripts/ros_exec.sh scripts/run_field_test.sh
git diff --check
```

主机全量：**123 passed, 5 skipped, 0 failed**，另有 8 个 unittest subtests 通过。
5 个跳过中，4 个是需显式启用的 ROS 集成用例（下方另行实际通过），1 个需要真实 BC bag metadata。

```bash
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --base-paths "$C1/src" "$REPO/agt_robot_studio" \
  --packages-select camera_gimbal_interfaces pantilt_camera_serial camera_gimbal_capability agt_robot_studio \
  --event-handlers console_direct+
colcon build --base-paths /workspace/work/agt_navigation_v3/interfaces/agt_robot_interfaces \
  --packages-select agt_robot_interfaces --event-handlers console_direct+
source install/setup.bash
QT_QPA_PLATFORM=offscreen colcon test --base-paths "$REPO/agt_robot_studio" \
  --packages-select agt_robot_studio --event-handlers console_direct+
colcon test-result --all --verbose
```

5 个不同包构建成功；Studio 经最终重建后 colcon 实际执行 **32 tests, 0 errors, 0 failures, 0 skipped**。
这 32 项属于主机套件的重复环境验证，不与主机通过数相加。

```bash
cd "$REPO"
AGT_RUN_ROS_INTEGRATION=1 ROS_DOMAIN_ID=188 QT_QPA_PLATFORM=offscreen \
  python3 -m pytest -q tests/ros --tb=short
```

真实 Humble 生成的消息与 Action、C1 真实能力节点配合软件设备夹具：**4 passed, 0 failed**。
覆盖保存真实编码图片/记录、图像超时、动作超时、PointCloud2+OccupancyGrid+时间戳 TF+Qt 三视图。
另外实际导入 `scripts/ros_bridge.py` 成功，Bunker/INS 可选消息包缺失不再阻断其他功能。

完整原生 bringup 构建也实际尝试：

```bash
colcon build --base-paths "$REPO/agt_field_test_bringup" \
  --packages-select agt_field_test_bringup --event-handlers console_direct+
```

结果：**1 package failed**，CMake 在 `find_package(PCL)` 处找不到 PCLConfig.cmake。
容器还没有完整的 Livox SDK/驱动、agt_pointcloud_core 和车辆/导航原生部署 overlay；没有宣称完成整套
SLAM/Nav2/CAN 的部署构建或集成。真实 BC bag 回放没有输入数据，未运行。

现有独立原生脚本 `humble_session_lifecycle.py`, `humble_fwmini_control.py`, `humble_fwmini_tf.py`,
`humble_time_align.py`, `ros_field_loop.py`, `ros_bridge_rpc.py` 未在这个不完整 overlay 中运行：分别依赖完整
bringup/TF guard、FW-mini 适配器及 yhs_can_interfaces、Livox 消息/时间对齐二进制、PCL 与 Bunker 消息包。
浏览器旧流程脚本未运行（当前未部署其完整 ROS 后端及 Playwright/Chromium 组合）。

## 下一步上车验收顺序

1. 核对依赖锁和既有精确补丁，构建完整车辆 overlay；验证单个 RSP、map→odom owner 和 CAN driver。
2. 无运动条件下独立启动 MID360S/IMU，核验频率、TF、mount/profile SHA；再独立启动相机，测 USB/串口、
   健康、限幅、实际角度、稳定到位、新图像与磁盘记录；拔插/超时后导航及雷达进程应保持。
3. 架空底盘，分别验收 Bunker 和 FW-mini 的正确 CAN/话题/里程计；验证禁用启动、运动许可、急停、
   遥控接管、指令超时停车和手动守护恢复，故障时不得保持旧运动意图。
4. 静态启动定位；检查唯一 TF 所有权及地图/PCD/资产身份。验证三视图独立缩放、拖动、居中、跟随、
   放大恢复，以及丢帧、旧数据、TF 中断时仍保留有效画面和明确错误状态。
5. 先停止定位后可改变配置；同配置启动导航应复用既有定位、底盘与传感器。验证手动守护释放、停止导航
   后恢复守护且定位继续运行。随后低速短目标、两点路线、到点拍照、取消/超时和异常退出。
6. 使用有可靠共同坐标证据的新 PCD 生成候选，与 ACCEPTED 基准比较；核对变化统计、JSON/PNG SHA、
   人工通过/拒绝、未自动激活、旧版本不变及新几何禁用旧资产。新地图正式导航仍需匹配的新安全验收报告。
7. 验收录包、路线保存、地图导出、参数保存、采集目录打开与包含任务元数据的 ZIP 导出。

这些项目当前全部为 `HARDWARE_VERIFIED: NOT_RUN`。
