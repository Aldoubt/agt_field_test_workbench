# Bunker 命令行 + RViz 现场测试

当前入口：`~/agt_field_test_ws/field_trial`。它自动加载 Humble 和当前工作区，不依赖 Studio 或 Workbench HTTP 会话。

5/7/10 米重复精度测试及指定多点云台拍照，见 [专项操作步骤](accuracy_and_photo_tasks.md)。
直接在 RViz 地图上选点排队拍照，见 [RViz 多点拍照](rviz_photo_tasks.md)，入口 `./field_photo`。

本次用户已确认标定实测可用；现场模式直接使用当前外参，不要求毫米级复测，也不要求先完成 PHYSICAL_CALIBRATION/PHYSICAL_NAVIGATION 正式验收报告。原标定不改写成 VERIFIED，不填造未做的 PASS。

保留与实际运行有关的检查：单一 LIO/TF/底盘控制链、地图与 PCD/SC 对应关系、真实轮速静止反馈、里程计时效、全局定位有效性、底盘反馈/故障和遥控优先权、命令超时停车。当前上限为前进 0.40 m/s、后退 0.15 m/s、角速度 0.50 rad/s，FAST-LIO 适配器 `max_input_age_sec=0.20` 不变。

## 下午直接使用

所有终端先进入工作区：

```bash
cd ~/agt_field_test_ws
```

终端 A 启动整条链路，保持运行（会启动真实传感器、底盘驱动、定位、Nav2 进程和 command guard）：

```bash
./field_trial start --hold-system-clock
```

终端 B 打开已配好图层和 Nav2 Goal 工具的 RViz：

```bash
./field_trial rviz
```

先在终端 C 仅激活静态地图，核对显示的是约 131.5 × 99.4 米的农科院大图：

```bash
./field_trial map
```

此命令只启动 map server，不激活 Nav2，不发送运动指令。

终端 C 在车停稳后请求 SC/KISS 全局重定位，查看真实状态：

```bash
./field_trial relocalize
./field_trial status --seconds 5
```

本地里程计应约 10 Hz，`localization.local_odom_fresh=true`、`global_correction_valid=true`，定位状态应为已定位，轮速和底盘反馈应持续收到。`matcher.status` 显示全局匹配进度或拒绝原因。`status` 同时输出里程计时间戳年龄的 P50/P95/最大值，全部为实际 ROS 时间差。

全局定位成功后，在终端 C 激活地图和 Nav2：

```bash
./field_trial nav
./field_trial status
```

`nav2.controller_server` 和 `nav2.bt_navigator` 应为 `active`。现在在 RViz 用 **Nav2 Goal** 拖出位置和朝向，从近距离目标开始测试。没有使用 `2D Pose Estimate`，全局位姿来自真实 SC/KISS 匹配。

Nav2 进程在 `start` 时启动，但生命周期自动激活关闭，避免全局定位前等待 `map → odom` 导致启动超时。`map` 可提前显示静态地图；定位成功后 `nav` 激活 Nav2，跳过已激活的 map server。尚无有效全局定位时，RViz 中机器人/局部数据缺少 map 变换仍会报错，但可以独立核对静态地图。

可选终端 D 录包，结束时在该终端 Ctrl+C 完成 bag：

```bash
./field_trial record
```

取消当前 RViz 导航目标：

```bash
./field_trial cancel
```

结束整个测试链路：在终端 A Ctrl+C；RViz 和录包终端分别 Ctrl+C。紧急处置继续使用实物急停或遥控器。

## 常见状态

| 状态/现象 | 处理 |
|---|---|
| `waiting_local_odom` | 检查 A 终端 FAST-LIO/适配器诊断及 `status` 中输入频率和 odom 年龄 |
| `waiting_global_pose` | 停车后执行 `relocalize` |
| `WAIT_STATIONARY` | 检查真实 `/wheel/odom` 是否新鲜、车辆是否停稳；不能用虚拟零轮速替代 |
| SC/KISS REJECTED/FAILED | 查看 matcher detail 和 A 终端日志，确认环境与当前地图对应后重新请求 |
| `controller_server`/`bt_navigator` 为 `unconfigured` | 定位成功后执行 `nav` |
| `nav` 报 `no_time_aligned_local_odom` / FAST-LIO `input_fault=1` | 先检查本地里程计；BBS 成功不能替代本地里程计。故障锁定的旧进程需由操作者停车后重启 start，重新静止初始化和重定位 |
| Nav2 active 但车辆不动 | 检查底盘 `control_mode`、`vehicle_state`、`error_code` 和 command guard 日志；遥控优先和反馈门禁仍生效 |
| `resource already owned` | 已有组件持有资源；在原终端/Studio 结束对应组件后重试，新入口不会擅自杀进程 |

### 有匹配箭头，但模型或膨胀层没有出现

匹配器 `SUCCEEDED` 表示后端输出了位姿；仍须实时定位状态 `global_correction_valid=true`。2026-10-25 现场出现过匹配约 1 秒成功，但 FAST-LIO 此前检测到 2.014 秒 IMU 历史缺口并锁定停发，定位管理器因此报 `no_time_aligned_local_odom`，不能建立有效 map→odom。此时 `status` 中 adapter 的 `publish_rate=0`、本地里程计缺失，不能仅凭地图上的箭头判断定位可用。

遇到上述输入故障，在原 start 终端 Ctrl+C 后由操作者重新 `./field_trial start --global-backend bbs`，保持静止完成 IMU 初始化，再请求重定位。不能通过清队列、修改时间戳或继续跨缺口预积分恢复旧滤波状态。若再次发生，保留 start 终端 `LIO_DIAG` 与传感器日志以检查缺口来源。

所选标定生成的机器人描述已加入现有 Bunker/MID360 CAD 外观，所有标定关节保持原值。RViz 的 RobotModel 读取 `/robot_description`。在 Fixed Frame=map 时，模型仍需要有效的 map→odom→base_link 变换。

膨胀层由 Nav2 costmap 插件生成，不属于静态地图；全局定位有效后执行 `./field_trial nav`。Global Costmap/Local Costmap 已配置 `costmap` 配色；更新配置后重开 RViz 或重新加载 `config/field_trial.rviz`，不会影响传感器/底盘。Nav2 面板的 Localization 指示通常针对 AMCL，当前 FAST-LIO/BBS 链路以 `field_trial status` 为准。

## 显式尝试 3D-BBS + GICP

SC 返回 SCAN_CONTEXT_NO_CANDIDATE 表示描述子检索没有满足阈值的候选，尚未进入 KISS/GICP，不能据此认定本地里程计或标定失败。CLI 支持显式选择 BBS，SC 不会自动切换成 BBS。

在原 start 终端 Ctrl+C 结束旧会话，然后启动同一张农科院大图、同一份标定和 FAST-LIO2：

```bash
./field_trial start --global-backend bbs --hold-system-clock
```

另一个终端确认地图，停车并请求匹配：

```bash
./field_trial map
./field_trial relocalize --timeout-sec 60
```

`--timeout-sec 60` 在请求之前设置匹配器 sdk_timeout_sec，调整计算预算，不放宽里程计时效或匹配质量检查。请求受理成功不代表匹配成功；允许它计算后查看：

```bash
./field_trial status --seconds 5
```

当前大图的 Polar Context 候选库已恢复，共 4912 条、跳过 0 条。BBS 请求现在使用 **Polar Context → candidate 3D-BBS → GICP**，状态中的 backend 应为 `polar_context_bbs_gicp`。此前库构建失败时使用的是 `whole_map_bbs_gicp`，整图搜索曾在约 60 秒超时。

已运行的 BBS 会话会在每次请求时重新检查候选库，不必为此次恢复重启。直接再次执行 `./field_trial relocalize --timeout-sec 60` 即可；请求受理不等于定位成功，仍需查看 `global_correction_valid`。恢复后，保存的现场查询离线匹配返回成功（score 0.907、fitness 0.1265、overlap 0.764），在线定位仍需实际请求验证。

恢复 SC/KISS 时，在原 start 终端结束 BBS 会话，再 `./field_trial start --global-backend sc_kiss`。`--global-backend` 也适用于 `check` 和 `start --dry-run`，不会修改配置默认值。

## 行驶中时间跳变与地下模型显示

现场曾在 14:15:39 发生电脑时钟回拨：systemd-resolved 同时记录 Clock change，FAST-LIO 出现 IMU 迟到和约 4 秒的未来扫描，随后空 IMU 包锁定停发。Livox 无设备时间同步时用主机系统时间给数据包打戳，点相对时间又转为 uint32；回拨可使负差回绕到接近 4.295 秒。当前没有原始故障帧 bag，证据为同步的系统日志、LIO 诊断与驱动源码。具体是谁发起了调时未在日志中识别。

当前主机用 systemd-timesyncd。现场启动建议加 `--hold-system-clock`：先确认系统已完成校时，启动期间暂时 stop 该服务，测试退出后 start 恢复；不修改当前时钟、不伪造消息时间戳。命令可能要求 sudo 密码，长会话退出后恢复也可能再次要求密码。其他时钟服务运行时该助手会拒绝操作，避免误称时钟已保持。人工调时仍应在测试退出后进行。若终端被强杀、无法正常清理，手动恢复 `sudo systemctl start systemd-timesyncd.service`。

前端现会在处理前拦截大于 0.20 秒的单帧点偏移（本工作流固定 10 Hz）和大于 0.10 秒的 IMU 倒退。对异常时间锁定停发，不清空队列后沿用旧滤波状态。修改时间保持参数不影响正在运行的旧进程；需由操作者重启本轮会话。

二维栅格平面 z=0 不代表三维地图地面高度。本图匹配出的 base_link z 约 −0.75 m，模型在斜视图中位于栅格下方是二者零高度不同。RViz 默认改为二维俯视，静态地图在底层绘制；真实 TF、PCD、地图版本和标定不变。重开 RViz 加载新配置即可。

## 线缆自体回波、局部窗口和提速

2026-10-25 静止实采确认：原自体盒只覆盖到 base_link z≈0.45 m，而车上固定线缆/支架回波位于 x=[−0.301,−0.053]、y=[−0.109,0.129]、z=[0.814,1.587] m。当前机器人配置增加独立小盒 `navigation_self_mask_boxes: [-0.36,0.02,-0.17,0.19,0.72,1.68]`，各组六个值依次为 xmin/xmax/ymin/ymax/zmin/zmax，坐标为 base_link。它在车辆平面轮廓内，只用于导航预处理，不扩大整个后向扇区，不接入 FAST-LIO、SC/KISS 或 BBS 查询支路。

捕获点云经过实际原生预处理器 A/B：41 个自体体素被去除，1634 个盒外体素全部保留。装置或线缆位置变化后应重新检查小盒，不能将环境点按静止持久性自动判为自体。旧代价地图中的残留随本次重启重建，不在有目标时清图绕过碰撞检测。

机器人配置统一下发前进 0.40 / 后退 0.15 m/s、转向 0.50 rad/s，线加速/减速 0.40/0.80 m/s²、角加速/减速 0.80/0.90 rad/s²；弯道、近障碍和接近终点时仍由 RPP 自动减速。控制器、平滑器和最终 guard 保持同一配置来源。

局部 8×8 m 窗口计算从 10 Hz 提升到 20 Hz，发布从 5 Hz 配为 10 Hz。原生控制器隔离测试实测约 20 Hz 计算、8 Hz 发布，原配置约 10 Hz 计算、4 Hz 发布；Humble 发布周期与更新周期取整会使实际发布低于标称。雷达和本地里程计仍为约 10 Hz，提高窗口频率不会产生新的传感器帧。全局大图频率、窗口大小、栅格精度、碰撞检测和膨胀半径均保留。

本次包含原生节点及启动时读取的频率参数，需要停车后结束旧 start 会话，再 `./field_trial start --global-backend bbs --hold-system-clock`。静止初始化、map、relocalize、status 均正常后再 nav。旧 RViz 关闭重开，里程计显示 Keep=1，去掉截图中的历史红色扇形箭头。

## 恢复或补建 Polar Context 候选库

```bash
./field_trial build-polar-context --dry-run
./field_trial build-polar-context
./field_trial check --global-backend bbs
```

命令使用所选地图关联的原始关键帧和 PCD；不启动设备、不请求定位、不发送运动指令。已有候选库直接复用。构建成功后应显示 `polar_context: READY`，并在检查中看到候选搜索链路。

本次故障是格式衔接遗漏：关键帧导出器写 `poses_timed.txt`（文件名、时间戳、位置、四元数），旧原生构建器只读八列的 `poses.txt`。原始 4912 帧及位姿没有丢失。现加入兼容转换，只去掉时间戳列，保留位姿数字及顺序，以符号链接复用原始点云；不改写原始文件。兼容输入和 SHA256 来源记录保存在对应 `relocalization/polar_context_source_*`。候选库先在临时目录构建，验证非空后才发布正式 DB，失败不会留下供运行匹配器读取的半成品。

保存地图时同时保存 `relocalization/polar_context.db`、`polar_context.yaml`、`asset_status.json` 和来源记录。重新构建还需要原始 `keyframes/patches` 与位姿文件，单个融合 PCD 不包含逐帧位姿。Workbench 初始化通常不会删除 maps/runs；仍应保留交付包，代码重置或重新检出后恢复其中的 `workbench/artifacts.py`、`workbench/polar_context.py` 和 CLI 文件，并先核对当前版本差异。

## 配置和诊断

当前固定配置 `config/field_trial.yaml` 选择地图 `20261025T023926Z-231ee788`（bunker 农科院 建图测试1 去残影候选，已 ACCEPTED，131.45 × 99.4 米），FAST-LIO2、对应的 4912 帧 SC 数据库和现有 MID360S JSON。启动前检查网格、PCD、SC 数据库关联，标定/运动配置冻结到 `runs/field_trial_<时间>_<id>/`，不修改原始文件。启动终端会明确打印地图名称、版本和尺寸。

2026-10-25 纠正了先前错误选择的 47.2 × 30.7 米局部图。正在运行的会话仍使用其冻结配置；修改这里的默认图不会热切换运行中的滤波器或全局位姿。请在原 start 终端 Ctrl+C 后重新 start，再请求重定位。

`status` 会补读本工作区当前匹配进程的最近一次日志事件，避免几秒采样错过一次性 REJECTED。此时明确标记 `cached=true`，不能将缓存成功当成当前定位有效。频率按第一条与最后一条实际接收消息的间隔计算，排除 DDS 发现等待时间；单条样本显示 null。

```bash
./field_trial check                 # 文件与关联检查，不启动设备
./field_trial start --dry-run       # 只看完整启动参数
./field_trial --help
```

更换地图可修改 `config/field_trial.yaml` 的 `map_id`，或在启动时指定 `--map-id <地图版本>`。其他终端的 nav/重定位/状态命令操作同一条运行链路，无需再次选图。

底层 launch 新增 `navigation_mode:=field_test supervised:=true`；原有 Workbench/Studio 正式导航入口维持原策略。下午按本文使用 CLI，不再被 CANDIDATE 标签和缺少正式报告阻断。

## 规划请求确认超时（2026-10-27）

现场 11:41:56 出现 `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`，随后跟随任务被取消；录包在报错后约 10 ms 仍收到新路径。运行中的默认确认时限为 20 ms。这是 Action 接单确认等待，不是规划计算时限或停车等待时限；未采集逐跳确认时序，尚不能确定具体调度/通信延迟来源。

普通导航、直线测试生成的行为树以及拍照行为树，现在为 ComputePathToPose / ComputePathThroughPoses 显式设置 `server_timeout="200"`（毫秒）。FollowPath、碰撞检测、停车等待和规划计算预算保持原值。200 ms 是待现场复测的有限确认预算，不代表已经解决所有延迟问题。停车结束旧会话后按原命令重新启动，以重新生成并加载行为树；仅修改 YAML 默认值不保证已构造的 BT 节点刷新。
