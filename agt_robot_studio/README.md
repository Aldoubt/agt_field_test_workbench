# AGT Robot Studio — 独立 ROS 2 Humble / Qt5 工作台（v0.1.1）

`agt_robot_studio` 是独立 `ament_python` ROS 包，与原有 V3/V4/Qt HMI/Web Workbench 解耦。风格沿用深色侧栏、大地图主窗与定距面板。**实车与 Humble 编译仍未验收；默认禁止动作下发。**

## 部署

```bash
cd ~/ros2_ws/src
git clone -b feature/agt-robot-studio-v1 https://github.com/Aldoubt/agt_field_test_workbench.git
sudo apt update
sudo apt install -y python3-pyqt5 python3-numpy python3-yaml ros-humble-nav2-msgs ros-humble-nav2-behaviors
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_robot_studio --symlink-install
source install/setup.bash
ros2 run agt_robot_studio agt_robot_studio --demo
# 真实 ROS 只读观察，默认 live_motion_enabled=false
ros2 run agt_robot_studio agt_robot_studio --config ~/ros2_ws/src/agt_field_test_workbench/agt_robot_studio/config/studio.yaml
```

## 改动 v0.1.1：**不再由 GUI 执行运动闭环**

旧 v0.1.0 在 GUI 的 ROS 线程里订阅点云、轮速里程计、两个自定义 Bool 并发 20 Hz Twist，`odom_max_age_s=0.35`。高负载可能误判或使 ROS 工作线程异常退出。**v0.1.1 已移除 GUI Twist 发布与这个 0.35s odom 硬门槛。**

- GUI 只读订阅 `/odom` 做显示，没有数据时显示“未收到”，不会杀死 Qt、GLIM 或 Nav2。
- GUI 点击“前进 / 后退距离” -> `nav2_msgs/action/DriveOnHeading`；点击“旋转” -> `nav2_msgs/action/Spin`。操作与目标发送均使用非阻塞 Action Client；结果/反馈异步显示。`停止` 请求 Action Cancel；**软件取消不是硬件急停**。
- Nav2 Behavior Server 在**独立进程**用 rolling `local_costmap/costmap_raw` 与 `local_costmap/published_footprint` 进行轨迹碰撞检查；原有 Obstacle/Voxel Layer 可以继续使用。无需 GUI 自造 `/agt/safety/obstacle_clear` 主题。
- GLIM 点云解码移到独立 daemon 线程，容量为 1 的队列只保留最新一帧；丢帧只影响显示，不延缓 ROS Action 通信。
- `std_msgs/Bool /agt/safety/motion_permitted` 仍作为可选 UI 的**严格准入门禁**：必须由已验证的独立底盘安全管理器周期发布，当前仓库不伪造 `true`。该 Bool **不是**最终的电机安全保护链；实车必须同时让底层速度管线按控制权、急停和心跳 fail-closed。
- 默认 `live_motion_enabled: false`，未验收不能开；默认 `allow_reverse: false`，后方盲区、净空和刹停距离未验收不开放自动倒车。

## 为什么已有局部 costmap 还要保护管线？

正常 Nav2 路径执行和 Behavior Server 的 Spin/DriveOnHeading 会主动查局部 costmap；但任意直接发送 `/cmd_vel` 的手动/底盘节点可绕过这些检查。速度输出的**最后一层**应当是 `nav2_collision_monitor`（基于实时点云/扫描直接监测 stop/slowdown，不依赖 costmap）及已经验收的 Motion Guard/底盘硬件 watchdog。这些不在本 GUI 内实现，也不能通过假 Bool 代替。

推荐（需现场验证实际 cmd_vel topic 与 TF 链）：

```text
Qt --DriveOnHeading/Spin--> nav2_behavior_server (LOCAL COSTMAP collision check)
      --raw cmd_vel--> velocity smoother / collision monitor --> Motion Guard
      --> hardware CAN driver with watchdog --> motor
```

确保唯一最终 `/cmd_vel` 驱动发布者，所有运动来源都经过相同过滤器。急停与遥控优先权必须由真实电控和 CAN 驱动保障。Nav2 避障是软件防护，**不是安全认证急停**。

## 在实车允许动作前核查

```bash
ros2 action list -t                 # /drive_on_heading, /spin 必须存在
ros2 lifecycle get /behavior_server  # 必须 active（若节点名不同请调整）
ros2 topic info /local_costmap/costmap_raw
ros2 topic info /local_costmap/published_footprint
ros2 topic info /agt/safety/motion_permitted
ros2 topic info /cmd_vel -v          # 不应存在绕开最后速度保护层的并行发布者
```

只有**真实** Motion Guard 发出的 `motion_permitted=true` 才能满足软件入口，此外必须正确检查 sensor TF、footprint、局部 costmap 活性、控制权/急停、后向盲区、驱动看门狗，经过模拟和架空轮等阶段后，才能把 `live_motion_enabled` 改为 true。当前版本**不提供伪造许可的 mock**。

## 地图与录包

- `GLIM` 实时 PointCloud2；点云浏览可旋转缩放；`ros2 bag record` 原始话题；PGM/YAML 地图编辑另存。
- RAW Bag 一定要记录 `/livox/lidar`、`/livox/imu`、`/odom`、`/tf`、`/tf_static`、`/cmd_vel`、原始 CAN 或轮速话题和 GLIM 轨迹。源头话题名以实机为准。
- `DriveOnHeading` 不能代替高精度真值：线速误差请结合现场尺量和 LIO 连续轨迹；发生局部里程计漂移时 Nav2 应取消动作/停止，而 Qt 保持在线。

## 测试

```bash
PYTHONPATH=. python3 -m pytest -q test
python3 -m compileall -q agt_robot_studio
```

这只是**源代码静态/逻辑测试**；没有 Humble GUI、行为服务器、局部地图、硬件运动验收的结论。
