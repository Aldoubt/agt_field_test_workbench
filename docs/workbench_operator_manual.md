# 双底盘工作台操作手册

日期：2026-10-23。日常入口：`scripts/run_workbench.sh`，浏览器 `127.0.0.1:8765`。先按 README 一次完成工控机部署。云端有 Humble 容器；此处未连接车辆或传感器。

## 1. 选择车辆与运行会话

1. **Robot Setup** 选择 Bunker 或 FW-mini，核对 profile 中速度、footprint、CAN 和话题。profile 位于工作区 `config/robots/<robot>.yaml`；旧配置会显示校验原因与迁移候选；确认编辑器内容后点“保存新配置并选用”，生成新版本和 active 指针，原 YAML 保留。速度/加速度、CAN 端口与 footprint 沿用现有数值，协议字段使用当前契约；footprint 仍需实测。现场调整后重新启动会话，运行时快照保存在 `runs/<session>/robot.yaml`。controller、smoother、guard、驱动使用同一数值来源。
2. **Calibration**：Bunker 选择独立标定包；FW-mini 选择安装外参版本。不要把厂家内部雷达/IMU 变换当成底盘安装变换。
3. 填写该车 MID360 IP JSON。需要工作台启动传感器时勾选“本会话启动 MID360”；外部已经启动时取消，避免重复驱动。传感器任务不启动底盘。
4. 运动前逐项确认线束、供电、CAN 500 kbit/s、实体急停、遥控接管、区域清空和有人监督。FW-mini 还须提供 `body.measured: true` 且与外参版本尺寸匹配的几何 YAML。
5. 任务选择：**传感器观察**、**低速受监督手动/建图**、**定位观察**、**已验收导航**。一次仅一个会话。先“停止会话”再换车、外参或任务。停止会取消任务并发零速；实体急停始终由现场人员掌握。

**默认不会自动选择 FW-mini 运动档。** FW-mini 先在 Navigation 点 4T4D（CtrlCmd 6）或横移（CtrlCmd 8），等待新鲜连续反馈到 READY。按住 jog 后显式请求启用；松开会禁用，下一次需重新选模式。6 档只允许 vx+wz，8 档只允许 vx+vy。禁止用 steering_ctrl_cmd 的 5/7 编码。

## 2. 标定与时间同步

### Bunker 标定包

工作区 `calibration/bunker/`，初始文件 `bunker_calibration_candidate.yaml`。页面 JSON 编辑器的保存产生新候选版本；保留原文件。

四部分独立：

- `mount`：`base_link→livox_frame`，实际生成 RSP 固定边；`base_footprint→base_link` 高度单独管理。
- `internal.fastlio2` 与 `internal.batch_lio`：各自原始约定，分别生成 FAST `r_il/t_il`、Batch `extrinsic_R/T`。原来的正负号分别保留，未完成方向实测前不互相复制。物理 IMU TF 明确按 FAST 解释求逆。
- `rtk_arm`：天线杆臂未测量时禁用，**不发布虚构的零杆臂 TF**。
- `time_sync`：算法输入经过 `time_align`，不是只记数字。约定 corrected=raw+offset；Livox header 与 timebase 同移，point.offset_time 不变。raw bag 仍录原始时间戳。

采集静止、原地旋转、直行、S 形/八字，每组重复；记录安装方向、序列号、旋转中心、参考坐标、残差和数据 SHA。用同一 bag 离线复验。当前基线锁定 `esti_il=false / extrinsic_est_en=false`。上游在线估计开关存在，但没有输出完整收敛轨迹，本轮没有制造“在线标定已完成”的结果；需独立标定实验输出收敛/重复性/重放证据后才固化。

时间检查：填写网卡，在 Calibration 点 **PTP/phc2sys 观察**，Runs 查看报告。工具只读取 timedatectl、pmc、phc2sys、网卡时间能力；命令缺失或硬件未接保持 NOT_RUN/WARN。必须另核对 MID360 PTP 模式、PHC/系统时钟、驱动与 bag 的 header/timebase，检查校正后的话题确实被算法订阅。软件常数偏移测试不证明物理同步精度。

### 升级 VERIFIED

候选仍可做传感器测试、标定录包、建图与受监督定位观察。Bunker 升级报告放在 `reports/` 或 `runs/`：

```json
{
  "scope": "PHYSICAL_CALIBRATION",
  "robot": "bunker", "sensor_id": "实际序列号",
  "calibration_sha256": "页面显示的候选内容 SHA",
  "operator": "操作者", "observed_at": "实际时间",
  "observations": "实测记录、bag、测量表与残差文件位置/SHA",
  "checks": {"static":"NOT_RUN", "rotation":"NOT_RUN", "straight":"NOT_RUN",
    "figure_eight":"NOT_RUN", "repeatability":"NOT_RUN", "offline_replay":"NOT_RUN",
    "time_sync":"NOT_RUN", "geometry":"NOT_RUN"}
}
```

只有真实完成的检查可改 PASS。升级按钮核对绑定和报告 SHA，另存 VERIFIED 版本；候选不覆盖。FW-mini 使用其独立报告契约，见 [FW-mini 外参手册](fwmini_mid360_integration.md)。原始 10 月 21～22 日候选数值、证据和待验证项保留。

## 3. 建图、录包和离线重建

1. 在 **Mapping** 选择 FAST-LIO2 无 PGO 默认后端。实机建图须先启动当前同车 sensors/manual 会话；raw recorder 确认订阅后才启动 LIO；传感器和 RSP 复用，不重复启动。
2. 实机模式自动录原始 `/livox/lidar` CustomMsg、`/livox/imu`。不要叠加 Recorder 的第二份 raw 录制。需要标定或摆动诊断时另用对应模板。
3. 现场用低速 jog 或遥控走完整区域；点 **完成建图并导出**。它先正常关闭 raw bag、导出 FAST 世界点云与轨迹，再转换为 DRAFT。等 Runs 结束后再停会话。
4. **取消/停止**用于异常终止，保留输入和日志；不保证有完整地图，不能把 INCOMPLETE 产物当成功导出。不要直接断电。
5. 离线指定同一原始 bag，选择新外参/内部配置后重建；每次新输出目录，保存 bag 文件 SHA、冻结参数 SHA、外参和源代码版本。
6. 无 PGO 路径是真实的 `lio_artifact` exporter，订阅 `/fastlio2/world_cloud` 与 `/fastlio2/lio_odom`，通过 `/mapping/fastlio/export` 导出；不依赖 PGO 服务或其目录。地图数值轴等同初始 LIO world，尚无 ENU 对齐。
7. PGO 仅作为显式离线实验选项，不用于默认实车建图，也不因 GPS 实验自动开启。

## 4. 全量 3D 清洗与 2D 编辑

**Map Studio** 中：

1. 导入完整 PCD；原文件和导入副本保留 SHA。3D 预览为采样显示，实际处理使用后端 PCL 读取完整 ASCII/binary/compressed PCD。
2. 参数化 ROI `crop`（保留范围）、`delete`（删除范围）、`voxel`（体素降采样），可以连续操作。点击保存得到新 cloud 版本，保留父版本和操作清单。检查 input/output 点数。
3. PCD→PGM 填分辨率、高度范围、ground_min、原点[x,y,yaw]、范围[宽,高]、min_points、占据/自由阈值。高度相对当前 LIO 地图轴，先检查地面高度，不能假设 z=0 总是地面。留空参数自动取完整 XY 包围盒；大于 1600 万栅格时调整分辨率/范围。
4. 有地面点且无占据命中的格子才置自由；未观测格子保持 unknown。不要为了路线可达把未知区域全部刷成自由。
5. 2D 支持缩放、平移、Center Robot、矩形擦除、障碍线、禁行区、撤销/重做。保存产生新版本和 keepout 数据；DRAFT 也可继续编辑、清洗、导出。
6. 原 PCD/PGM 和 raw bag 不覆盖。地图版本绑定来源 PCD SHA、转换参数、标定和 bag 线索。正式导航还要检查地图身份、内容审核、footprint 与安全报告。

## 5. 路线、距离目标与定位

1. **Navigation** 选择地图，在画布点击添加航点，拖动移动，删除/排序，设置 yaw，保存新路线版本。每条路线绑定地图 ID、PGM/YAML SHA、map frame。地图变化须另存路线，不能静默复用。
2. 路线绘制可用 DRAFT；正式执行须 ACCEPTED 地图、VERIFIED 外参和 PHYSICAL_NAVIGATION 报告。报告按 `workbench/safety.py` 契约绑定 robot/profile/map/calibration SHA，并有 operator、observed_at、observations；physical_estop、remote_takeover、command_timeout、feedback_freshness、geometry、localization、tf_ownership、motion_safety 均需实测 PASS。
3. 选择**定位观察**先检查重定位与持续校正：点“请求全局重定位”，观察 BBS + small_gicp 候选、manager 状态和 TF。默认 Tracker 开；只有 manager 发布 map→odom，候选必须经过现有质量/步长门控。
4. 满足所有门禁后启动**已验收导航**。页面须显示可靠新鲜定位、底盘控制权与反馈、Nav2 controller/bt_navigator ACTIVE。输入“向前 1 m/5 m”或后退距离，按当前 map 中车头方向生成目标位姿，经 NavigateToPose 规划跟踪。后退表示目标在后方，现有 Planner/RPP 可能先转向；它不是强制倒车 jog。
5. 多点使用 FollowWaypoints；显示机器人、目标、规划路径、局部 costmap、定位和 action 结果。取消会请求 action 取消并零速，运行门禁失效同样取消。
6. FW-mini RPP 导航只接受 4T4D 6 档，横移自主控制器尚未验收，因此 8 档只开放受监督 jog。任何模式/来源切换先零速并重新选档、等待反馈，旧指令不恢复。
7. 拍照 checkbox 只预留 waypoint task executor 绑定；当前没有相机执行器验收证据，拍照路线会明确拒绝执行。不能把“到达航点”记作“拍照成功”。

## 6. KISS 与 Tracker 离线 A/B

KISS 可选构建：在 Humble 中运行 `python3 scripts/build_kiss_optional.py`（额外安装 python3-pip；不影响基础 build）。锁定主 SHA 和 Python 构建依赖，独立安装到 `work/kiss_python_locked`，不改全局 ROS Python 包。

在 **诊断与验收 → 离线实验** 填 benchmark JSON：同一 map PCD、同一批 query PCD、共同 calibration SHA，`query_frame: mapping_body`。query 必须有可追踪的同一 bag/时间窗/外参转换来源；工具不会猜测已在地图坐标系中的扫描。推荐静止 query 窗口；运动扫描必须使用相同去畸变流程。

```json
{"query_frame":"mapping_body", "calibration_sha256":"共同外参SHA",
 "map":"/absolute/map.pcd", "timeout_sec":15, "top_k":8,
 "tile_width_m":30, "tile_stride_m":15,
 "queries":[{"pcd":"/absolute/query.pcd"}]}
```

KISS 接口：重叠子地图生成 → 中心径向直方图确定性排序（不是已验证地点数据库）→ KISS 全局配准 → 同一锁定 small_gicp native 精配准 → fitness/overlap/重复行歧义门控。保存所有候选和原因。检索召回率尚未真实验证，top_k 可能漏掉正确子图，正式对照仍为 BBS。

有独立真值时每个 query 增加 `ground_truth: {xyz:[...],yaw:...}`、`truth_evidence`、`truth_evidence_sha256`。没有这些不计算真实误差。合成结果不计真实成功率；真实误接受率、头尾歧义、恢复和 map→odom 连续性须另录证据。

Tracker A/B 按钮使用同一地图、bag、外参分别关闭/开启 Tracker，在 Domain 91 顺序运行，只回放原始传感器与 clock，不回放原 bag 的 TF/底盘命令，不启底盘。输出 observations bag、运行日志和诊断。请勿同时在 Domain 91 运行其他节点。无真实 bag 时本轮此项 NOT_RUN。

## 7. GPS/RTK 观察与第一阶段先验

Robot Setup 可选填写串口和波特率，点 **启动 INS 观察**。使用已构建的 `agt_asensing_driver`，固定正式消息 `/ins/navsatfix`、`/ins/status`、`/ins/imu`；不是用 /ins/odom 或 /ins/velocity 当真值。停止用 Runs 中对应任务的停止按钮。

先录质量、固定解、有效位、ENU 位置标准差和时间戳，确认实际硬件。未验证地图 origin、heading 和杆臂，GPS weight=0，仅观察。有信号不等于固定解。

可选 GPS 对齐 JSON 必须绑定当前 map/calibration SHA、origin_lla、enu_to_map_yaw/xy、lever_arm_base_xyz、VERIFIED 状态与 PHYSICAL_GEOREFERENCE 报告；报告校验 map_origin/map_heading/lever_arm/fixed_solution/timestamps/repeatability，详见 `agt_field_test_bringup/gps.py:load_alignment`。留空不开 GPS gate。

启用后全局候选先进入独立 GPS gate，质量有效时检查位置先验；GPS 退化移除约束，已定位时 GPS 创新跳变不用于修正。该节点不广播 TF，manager 仍唯一发布 map→odom。GPS 图优化是后续实验，本轮未实现/未启用，不开启 PGO。

## 8. Bunker 摆动分阶段诊断

Recorder 选 `motion_diagnostic`，记录命令每一环、wheel/LIO、TF、原始传感器、定位/Tracker/costmap/footprint/CAN，以及存在的 INS 三话题；缺失项保留清单。

先静止 → 人工低速直行 → 人工转向 → Nav2 直线 → Nav2 弧线。每阶段新 bag、一次改变一个参数。诊断页选择 bag 分析，Runs 给出输出目录，加载 diagnostic.json 查看同时间轴曲线。测量角速度从 LIO pose 的连续航向导数得到，保留原 twist 作对照；输出指令/响应延迟、摆动频率、wheel-LIO 航向差、map→odom 修正步长、横向偏差、时间差与数据缺口。没有相应话题的指标为 null/NOT_RUN。

分类只是线索：先排时间/LIO/TF，再比较底盘响应与控制指令、costmap/全局修正时刻；不能凭一条 FFT 峰直接判控制器故障。

## 9. 现场验收与报告

每次记录 robot、代码 SHA、profile/外参/地图/路线版本、operator、时间、环境、测量方法和证据 SHA。

- 架轮零速先核对 CAN 输出和档位反馈、驻车/禁用、遥控接管、急停和命令超时，确认真实制动；无实机记录为 NOT_RUN。
- 安全条件有效后落地低速监督：直线 → 转向 → FW-mini 横移。每项停稳后检查旧速度不能恢复，再进入下一项。软件零速不等于物理制动。
- 有安全导航条件后 5/7/10 m 各 5 次，采用尺量/测距仪/测量仪真值，正式平均误差 ≤12/25/35 cm；原工程 ≤12/15/20 cm 同时保存。不能用定位系统自身误差充当真值。
- 验收输入 trials 每项：distance_m、goal_xy、measured_xy、truth_source（physical_ruler/laser_rangefinder/survey）、evidence、evidence_sha256。
- 花穗/果实识别输入 GT 与 prediction 的 image/class/box[x1,y1,x2,y2]，prediction 另有 score；images 为路径→SHA，model 含 version/SHA。AP50 使用 IoU=.5、101 点插值，mAP50 ≥.80。保留图片、人工真值与模型文件，不接受只有最终百分数。
- 多点拍照需真正配置/验收相机 task executor，记录遗漏、图像质量和逐点任务结果；当前 NOT_RUN。

正式论文原件在当前环境未找到；目前阈值依据本次用户指令，收到原件后复核其测试方法和指标定义。软件、bag、实车状态分列，不自动把候选升级 VERIFIED。
