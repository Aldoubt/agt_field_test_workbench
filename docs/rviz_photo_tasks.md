# 在 RViz 中指定多点并逐点拍照

入口 `~/agt_field_test_ws/field_photo` 复用已有 `agt_rviz_patrol` 和 `mission_runtime`，不启动第二套底盘、定位或 Nav2。它自动加载当前 Humble 工作区。

2026-10-25 接线后实测：云台串口/角度反馈在线；相机 1920×1080，接收约 29.6 Hz，健康 state=1。已保存真实相机画面。没有下发车辆导航目标，也没有转动云台；实际多点行驶、云台转角拍照仍待操作者执行。

组合启动可直接使用 `./field_start photo`，原分终端命令继续可用，见 [启动模式](launch_modes.md)。两种方式都保留 `./field_photo start` 作为唯一显式队列任务触发命令。

## 启动

先按原流程 `./field_trial start --global-backend bbs --hold-system-clock`，静止初始化后 map、relocalize、status、nav，确认定位有效、Nav2 active。不要再启动其他整机 launch。

以下每一条分别在一个终端前台运行，终端都先 `cd ~/agt_field_test_ws`。已有相同组件运行时复用，不重复启动：

```bash
./field_photo camera
```

```bash
./field_photo runtime
```

```bash
./field_photo queue
```

```bash
./field_photo rviz
```

`camera` 使用当前 Wasintek 相机和 CH340 云台串口。可用 `--camera-device /实际设备`、`--gimbal-port /实际串口` 覆盖。旧版 usb_cam 对 `/dev/v4l/by-id` 的相对软链接处理有误，新入口会先解析成 `/dev/video0` 等实际设备路径。没有内参 YAML 会提示警告，普通导航到点拍照不依赖内参标定；图像仍须实际可用。

`queue` 默认读取 `config/field_trial.yaml` 的地图 ID。如果本轮 start 显式选了其他地图，queue 也要加同一个 `--map-id`。

关闭原来的导航 RViz，使用新的拍照 RViz，避免工具混用。此界面保留地图、模型、膨胀层等显示，增加 Photo Waypoints 和 Gimbal Camera；移除了会立即下发导航目标的 Nav2 Goal 工具。Navigation 2 面板保留，用于查看导航在线状态；拍照任务用 `field_photo start` 开始，面板内的普通导航按钮不会执行拍照队列。

已运行的旧 RViz 若看不到导航状态，可直接用菜单 Panels → Add New Panel → nav2_rviz_plugins/Navigation 2 加回来，不需要重启整机。拖动面板标题可停靠到右侧。后续 `./field_photo rviz` 默认加载此面板。原生面板的 Localization 检查通常针对 AMCL；本项目 FAST-LIO/BBS 定位有效性仍以 `field_trial status` 为准。

## 选点与执行

1. 确认 Fixed Frame 为 map，Gimbal Camera 能看到实时图像。
2. 选择工具栏 **2D Goal Pose**，在希望停车的位置按住左键拖出箭头，松开。拍照模式只要求停车位置在 0.35 米内，不强制底盘对齐箭头朝向；箭头朝向仍保存在任务文件。云台按自己的三视图模板转动。
3. 重新选择该工具，依次添加第二、第三个点。点进入 `/agt/rviz_patrol/goal` 队列，地图上显示绿色箭头，queue 终端显示 `QUEUED 1/2/3 waypoint(s)`；**此时车辆不应开始行驶**。
4. 核对顺序和路径区域，建议先选一个近点验证。操作终端执行下面命令才提交任务：

```bash
./field_photo start
```

不要用 Publish Point 添加拍照点：默认 `/clicked_point` 不接入该队列，也不包含停车朝向。若 2D Goal Pose 没有入队，展开左下角 Tool Properties → 2D Goal Pose，确认 Topic 为 `/agt/rviz_patrol/goal`，同时确认 queue 终端仍在运行。

流程：

```text
P001 → Nav2 到点 → 实测停稳 → 左/中/右三张照片 → 云台反馈确认回中
P002 → Nav2 到点 → 实测停稳 → 左/中/右三张照片 → 云台反馈确认回中
P003 → Nav2 到点 → 实测停稳 → 左/中/右三张照片 → 云台反馈确认回中
RETURN_HOME → 回到点击 start 时的位姿 → 停稳
```

默认每点云台 heading=-30/0/+30 度、pitch=0 度，角度相对于云台自身参考系，不是地图航向。可先查看画面确定安装零位；固定模板在 `src/agt_field_test_workbench/config/photo_three_views.yaml`。每次 start 重新读取模板。当前队列按同一三视图模板拍摄，各点不同角度的任务仍可使用上一节任务生成工具和 mission.yaml。

普通现场拍照现在使用 3° 到位容差、连续 2 帧反馈；到位后静置 0.2 秒，底盘实测停稳后额外静置 0.3 秒。实测停车检查本身保持不变，云台 8 秒运动超时不增加。此模板不用于测量云台角度精度。16:39 任务在左视图成功后回中位超时，记录最大误差 2.47°，超过旧 1.5° 要求；调整基于此现场反馈，仍会拒绝明显未到位的角度。

`field_photo start` 会先查询当前运行的 runtime、controller 和 global_costmap 参数：旧 runtime 未加载拍照行为树，或全局地图未加载实时障碍与未知区保护时，直接说明需重启。检查不切换遥控模式，也不发送车辆目标。

16:39 任务之后若导航仍是 16:37 启动的进程，日志已确认加载 `photo_goal_checker`，**只需重启辅助的 runtime 和 camera，无需再重启底盘、雷达、定位或 Nav2**。保持遥控模式，旧任务终止后，在各自原终端 Ctrl+C，再分别运行：

```bash
./field_photo runtime
```

```bash
./field_photo camera
```

camera 必须在 `field_trial start --hold-system-clock` 已稳定运行之后启动。现有相机进程早于导航时钟保持会话，图像时间戳实测约超前 0.41 秒；usb_cam 在启动时固定计算 monotonic 到 epoch 的换算，重启相机后应重新检查图像时间与位姿关联。保留驱动原始图像时间戳，不用当前时间覆盖旧图片。未重新实测前不宣称此时间问题已解决。queue 和 RViz 可复用，模板在下次新建任务时读取；旧任务文件的参数不会自动改变。

17:00 后的现场调整：到达范围由 0.25 米增至 0.35 米。16:56 任务留下的轨迹距目标最近约 0.2518 米，和旧门槛仅差约 1.8 毫米；操作者确认是车先停住、相机不动，随后才遥控接管。此次只在线更新 photo_goal_checker.xy_goal_tolerance=0.35，并已读回确认，不需重启整机或辅助进程；旧失败任务不会自动恢复。新建单点任务验证后再执行多点。

2026-10-25 拍照导航修正：`field_photo runtime` 显式加载 `config/photo_navigate.xml`，选择独立的 `photo_goal_checker`（位置 0.35 米，航向不限制）。该行为树保留路径失效后重新规划和碰撞检测；导航失败直接终止任务，不原地旋转、不倒车恢复、不自动清空代价地图。到点后仍须通过实测停车检查，随后才调用云台。行驶中为沿路径前进而进行的底盘转向仍正常存在。RETURN_HOME 同样只要求返回起点附近，不强制恢复最初航向。

普通 Nav2 导航和距离精度测试继续使用 `general_goal_checker` 的 0.08 米 / 5° 到达条件。Humble 在同时加载两个 checker 时要求行为树明确指定 checker；生成的普通单点、多点行为树均显式选择原 checker，避免影响其他导航入口。

旧现场进程不会自动载入新参数。应用这次修正时，先确认上一任务已经终止且车辆停稳，在原 `field_photo runtime` 和 `field_trial start` 终端分别 Ctrl+C，然后重新运行：

```bash
# 原 start 终端
./field_trial start --global-backend bbs --hold-system-clock
```

另一个终端按顺序执行；重定位是异步操作，status 确认 `global_correction_valid=true` 且本地里程计新鲜后再 nav：

```bash
./field_trial map
./field_trial relocalize
./field_trial status --seconds 5
./field_trial nav
```

在原 runtime 终端重新执行 `./field_photo runtime`。相机、queue、RViz 若仍运行可继续复用。重新排队前用 `./field_photo clear` 清除未提交的旧点，先以一个近点验证到点停稳和云台三张照片，再测试三点。不要直接重放上次失败的远距离任务。

start 返回 submitted 只表示请求已提交。最终 queue 终端应显示 `MISSION DONE success=True`。三个拍照点成功后应有 9 张图，completed=4 包括 RETURN_HOME。运行中不要向原 Nav2 Goal 工具或其他客户端再发目标。

选错了，在未开始任务时清空重选：

```bash
./field_photo clear
```

取消执行中的任务：

```bash
./field_photo cancel
```

该命令请求取消父巡检任务，执行器取消子 Action 并等待停车确认；受理响应不等于车辆已停稳。物理急停/遥控接管方式保留。

任务受理后队列清空；下次可以重新选点，已生成的任务文件保存在 `missions/rviz_photo/`，可用原 ExecuteInspectionMission 接口再次执行。

## 结果

### 实时障碍与有界重规划

2026-10-28 补全 Bunker 的全局实时障碍输入：全局代价地图按静态层、实时障碍层、膨胀层叠加，Smac2D 和 `IsPathValid` 均能看到新出现的车辆等障碍。标记输入为 `/agt/navigation/points_obstacles`（12 米），清障使用 `/agt/livox/points` 的原始射线（15 米）；静态地图占据和未知区域保持不可通行，不通过超时自动删除障碍。Bunker 的普通导航及距离测试共用此障碍输入，距离任务原有的失败终止策略保留。

拍照任务在开始和重试时明确请求新路径，行驶中每秒检查路径有效性。局部碰撞检查若先于全局更新触发，先结束本次跟踪、原地等待 1 秒，再重新规划和跟踪；每个导航目标最多恢复一次。持续碰撞、持续无路或等待失败最终中止任务，等待期间取消会阻止后续重试。不执行旋转、倒车或清空代价地图。控制器仍为 Planar RPP，全局规划器仍为 Smac2D，车体轮廓、速度上限及安装外参沿用当前配置。

此变化需重新启动原 `./field_start photo` 会话才会加载全局障碍插件和行为树。先结束旧任务并确认车辆停稳，再在原会话 Ctrl+C、重新运行启动命令并等待定位就绪。首次实车复测使用可接管的开放区域与静止障碍，确认新障碍进入全局代价地图、路径确实更新、停车净距足够后再扩大任务。隔离验证不能代替实车感知和制动验收。

### 规划请求确认超时与录包

2026-10-28 的 `rviz_patrol_20261028_153804` 完成 P001 三张照片和云台回中后，P002 在约 200 毫秒内因 `compute_path_to_pose` 未确认接收目标而终止。拍照行为树的 `ComputePathToPose.server_timeout` 已从 200 改为 2000 毫秒。这是 Action 请求确认的等待上限；导航任务的总等待仍为 300 秒，持续规划失败在上述一次恢复后终止任务。尚未通过实车任务确认延长等待能消除该问题。

Humble Nav2 会缓存相同路径的行为树。修改 XML 后，需在旧任务结束、车辆停稳后结束原 `./field_start photo` 会话，再重新执行 `./field_start photo`，等待初始化和定位就绪。单独重启 `./field_photo runtime` 不会清除 Nav2 内部缓存。分终端启动方式需重启原 `field_trial start` 导航链并按启动章节恢复定位与 Nav2。仅调整 XML 超时无需编译；全局未知区保护插件需先构建 `agt_field_test_bringup`。

复测前另开终端录包，保持它覆盖任务开始、点间切换和失败后数秒：

```bash
cd ~/agt_field_test_ws
./field_trial record
```

复测结束后在录包终端 Ctrl+C，等待正常退出。包保存在 `bags/field_trial_<时间>_<标识>/`。此命令包含雷达/IMU、里程计、TF、地图与代价地图、路径、定位/任务状态、`/rosout`，以及导航、规划和控制 Action 的状态/反馈；同时记录拍照队列目标与状态。不要重复启动第二个录包进程。

分析时同时保留 `missions/rviz_photo/` 下对应任务 YAML、`data/captures/missions/` 下对应任务记录，以及 `~/.ros/log/` 下同一启动会话的日志。Humble 常规 rosbag 记录的是话题，Action 的 `send_goal` 服务请求/应答不在其中，因此回放可还原输入和状态，但不能保证重现由调度或通信时序造成的确认延迟。确认超时应结合 `bt_navigator`、`planner_server` 日志与任务 `events.jsonl` 对齐分析。

图片和同步记录在：

```text
data/captures/missions/rviz_patrol_<时间>_<记录时间>/
  mission.yaml
  images/P001/left.png, center.png, right.png
  images/P002/...
  captures.csv
  captures.jsonl
  events.jsonl
  manifest.json
```

在实际记录目录上检查三点、每点三张：

```bash
ros2 run agt_navigation_runtime validate_records /实际任务记录目录 \
  --expected-points 3 --views-per-point 3
```

同时实际查看图像的清晰度和构图。记录校验不会代替图像质量检查。RTK 不作为本次拍照任务前提。

本轮软件验证：10 项测试通过，包含私有 ROS 域中的实际 PoseStamped 排队、顺序与朝向保留、生成三点加 RETURN_HOME 的任务、三视图模板和清空队列。未发送任何 Action 目标。证据 `runs/rviz_photo_20261025/`，其中 `live_camera.jpg` 为现场取图，`camera_receive_only.json` 为实测状态。

后续到达修正共 55 项运行时/现场入口/并发等待测试通过，另有 1 项行为树测试覆盖实际 Humble BT navigator 配合模拟 planner/controller 的成功、失败测试。模拟完整三点任务产生九张测试图片、四个导航目标（包括返航）；导航首点失败时不调用相机、不发后续目标。模拟图片不代表实车成像成功。证据 `runs/photo_arrival_20261025/`，现场新配置的完整三点行驶与云台动作仍待操作者验证。

运行时等待器改为复用一个 steady 计时器，避免 Humble 多线程执行器在频繁创建/销毁计时器时触发 InvalidHandle；关闭时仍唤醒等待者，取消和超时仍使用 steady 时间。测试另含 20 路并发、共 2000 次等待。

依赖仍锁定原版本；`patches/photo_mission.patch` 与 `photo_mission_manifest.json` 保存 runtime 修改，Workbench 初始化和构建脚本自动按哈希检查并复用补丁，遇到其他本地修改会保留并报错。

退出时先取消并确认正在运行的任务终止，再在各个辅助终端 Ctrl+C。它们不会停止原来的 field_trial 底盘/定位进程。

## 局部窗口拖影与面板 inactive

2026-10-25 后续排查：窗口实测约20 Hz计算、7.7 Hz发布；停车时原点不动正常。旧障碍不消失需要核查自体回波和清障射线，不能单凭画面说节点停机。实测controller/navigator/planner/local_costmap均active，原生Navigation2面板只在初始化时读取状态，先开RViz后CLI激活可能留下旧inactive；其Localization项默认查询AMCL管理器，不代表FAST-LIO/BBS状态。导航激活后关闭再添加Navigation2面板，或重新打开RViz以刷新；实际状态用field_trial status。不要为刷新显示连续点Startup。

确认过的车后线缆/支架过滤盒和局部体素odom高度范围已修正并通过原生隔离回放。它们需要在车辆停稳、任务终止后由操作者重启原field_trial start启动链加载，电脑不需要重启；尚未重启时不要把源码修正视作现场已生效。此启动会新建局部地图历史。本次没有擅自清空代价地图或执行现场导航。高度范围和自体过滤测试详情见 reports/rolling_window_20261025.md。未观测区域不会仅因时间流逝而清空。
