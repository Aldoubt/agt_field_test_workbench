# 2026-10-24 云端 Issue 集成交付（候选版本）

基线：`Aldoubt/agt_field_test_workbench` 的
`feature/agt-robot-studio-v1@b431aef243cb5cf535c90c699482ba6ad172fe66`。
本次未覆盖该分支或 main；集成分支：
`feature/agt-field-integration-20261024`，草稿 PR #6。
逻辑子 PR：#4（动态地图清理），#5（设备与 Qt）。
关联 Issue：#1/#2/#3，全部保持 OPEN。

## 已完成的软件范围

1. `agt_dynamic_map_cleaner`：独立 ament_python 包，基于既有
   `temporal_cleanup/keyframes/{poses_timed.txt,patches/}` 加载 body-frame
   逐帧点云和地图位姿；只在起点附近寻找“早期高位占据、晚于末次占据的
   多视角穿透射线”自由空间证据。输出新静态 PCD 候选、移除点和哈希 manifest。
   缺少证据不会推断成“人”，不会直接删除周围树干、墙面或地面。
2. 显式可选的建图保存阶段 `START_CLEARANCE`。
   默认 `AGT_START_CLEARANCE_CANDIDATES=0`，保持原先 C++ 时序清理、
   原始 PCD→PGM/BBS 的数据路径不变。设置为 1 才额外生成待审核候选；
   **即使生成成功，导入地图仍使用第一阶段静态 PCD**，不会自动激活候选。
3. Qt 新增「设备与传感器」页面，可通过现有会话启动 MID360S/IMU，
   有显示的五项底盘前置确认，C1 云台通过新的单独 Workbench Job 启停，
   并在全局状态栏报告它自己的**进程状态（不冒充图像 READY）**。
   C1 进程失败不会连带终止 Nav2 Job；相机真实 Action 拍照/图像订阅尚未验证。
4. Qt 导航地图支持两次点击指定位置和 yaw（第二次只决定航向）、
   方向线预览及 Esc 取消，保留已有航点/路线接口；修复 QComboBox
   弹出框白底白字问题。

GitHub Actions 的无 ROS Python 回归与 Qt offscreen smoke 已通过。
**未做目标机 colcon C++/ROS 2 原生构建、真实 bag 二次清理、C1 相机和底盘实车验收。**

## 工控机合并前只读检查与构建

当前机器的未提交修改、标定、地图和 bags 都需要保留，先核对差异：

```bash
cd ~/agt_field_test_ws/src/agt_field_test_workbench
git status --short
git fetch origin feature/agt-field-integration-20261024
git branch -r --contains origin/feature/agt-field-integration-20261024
# 工作树干净时才按需切换；不要使用 reset --hard
git switch -c field-issues-review --track origin/feature/agt-field-integration-20261024
cd ~/agt_field_test_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select \
    agt_dynamic_map_cleaner agt_robot_studio agt_field_test_bringup \
    --packages-ignore ugv_sdk bunker_base --symlink-install
source install/setup.bash
ros2 pkg executables agt_dynamic_map_cleaner
```

若本机已有正在运行的完整现场 ROS 会话，先完成保存/录包、停止并验收数据再
更新工作区；不要在地图保存期间更换软件版本。

## 起点人员清理离线试验（不启动车轮）

优先选今天确有“起点站人，随后离开，后来能看到该处自由空间”的 bag。
前提是已有 `temporal_cleanup`，其 keyframes 必须完整有效：

```bash
RUN=~/agt_field_test_ws/maps/<run_id>/temporal_cleanup
ros2 run agt_dynamic_map_cleaner clean_start \
  --static-pcd "$RUN/clouds/static_map.pcd" \
  --keyframes "$RUN/keyframes" \
  --cleanup-manifest "$RUN/manifest.json" \
  --config ~/agt_field_test_ws/src/agt_field_test_workbench/agt_dynamic_map_cleaner/config/start_clearance.yaml \
  --output "$RUN/start_clearance_candidate"
```

结果：`start_clearance_candidate/static_candidate.pcd`、
`removed_candidates.pcd`、`manifest.json`。同一路径不允许覆盖。
如果人一直站到结束、扫描未看到人离开后的原位置、或多视角证据不足，
结果可能是 `INSUFFICIENT_EVIDENCE`，属于保守行为，不可自动强制删除。

在三维视图/离线脚本核对后，必须比较静止人员残影面积和起点永久立柱/树干
保留率，手动 QA 后才考虑从候选 PCD 生成新的 PGM/YAML 与 BBS 同源版本。
任何误删静态结构都不能升级为导航 ACCEPTED。

**可选保存时生成候选**：在启动 Workbench 服务的进程环境中设置
`AGT_START_CLEARANCE_CANDIDATES=1`；若服务已经运行，则新开 Qt 终端
不会给已有服务添加此环境变量。不要为了切环境变量强停正在保存的会话。

## 设备层现场验收

- 传感器页：先只启动 MID360S/IMU，校验实际 LiDAR/IMU 话题频率；
  再检查 recorder 与建图自动录包不会相互重复。
- 手动底盘：确认五项安全前置检查。CAN 新鲜度、实体急停、遥控抢占和
  控制权仍由原有后端校验；GUI 打勾绝不等于允许运动。
- C1 云台：`/dev/video*` 和云台串口必须真实存在，并安装
  `autolabor_c1_bringup`。启动时仅可证明 Job 进程存在，
  必须进一步检查真实图像帧率、云台应答/范围及拍照动作。
- 正在运行导航时拔掉相机线，验证 camera Job 状态转为 FAILED，
  而原 Nav2/FAST-LIO2/底盘 Job 不被 Workbench 自动关闭。
  若路线任务要求拍照失败，应按明确的任务策略处理拍照动作，不可无限等待。
- 导航地图两次点击位置/yaw 的几何、保存/重载一致性仍要真人 GUI 验收。
- 现在**尚未实现**独立云台角度/拍照执行端到端、固定图片导出路径、
  三视图自由放大跟随、实时 2D 栅格预览、不中断驱动的底盘模式热切换、
  地图跨版本自动合并。这些继续留在 Issue #1/#2，不能标为 PASS。
