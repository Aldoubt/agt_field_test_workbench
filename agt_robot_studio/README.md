# AGT Robot Studio — 独立 ROS 2 Humble / Qt5 工作台

本仓库的 `agt_robot_studio/` 是独立 `ament_python` 包，不依赖 V3/V4、老 Qt HMI 或旧网页工作台。支持运行在 Ubuntu 22.04 + ROS 2 Humble。**实车未验收，默认不下发底盘速度**。

## 安装

```bash
# 从本仓库独立分支下载到工作区的 src 目录（仓库根目录不是 ROS 包）
cd ~/ros2_ws/src
git clone -b feature/agt-robot-studio-v1 https://github.com/Aldoubt/agt_field_test_workbench.git
sudo apt update
sudo apt install -y python3-pyqt5 python3-numpy python3-yaml python3-pytest
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_robot_studio --symlink-install
source install/setup.bash
ros2 run agt_robot_studio agt_robot_studio --demo
# 之后真实模式
ros2 run agt_robot_studio agt_robot_studio --config ~/ros2_ws/src/agt_field_test_workbench/agt_robot_studio/config/studio.yaml
```

也可只复制 `agt_robot_studio/` 文件夹到任意工作区 `src`，不需要构建同仓库其他包。

## 功能
- 深色 Qt 界面：左侧流程菜单、中央三维投影点云 (ROS `PointCloud2`) / 二维 `OccupancyGrid`、下方标定面板；鼠标拖动旋转点云、滚轮缩放；显示实时里程计和状态。
- 订阅 GLIM RViz Viewer 发布的 `/glim_rosnode/map`（全图）和 `/glim_rosnode/aligned_points`（实时）；前者视实际安装可能需修改。支持独立打开官方 RViz 3D。
- 通过 `QProcess` 启停 GLIM `glim_rosnode`、原始 Rosbag2 录制。GLIM 启动需要明确配置目录，**不会替你生成不可信 LiDAR/IMU 外参**。
- 编辑已经加载的二维 occupancy 栅格：障碍/空闲/未知画笔、撤销，另存新目录生成 Nav2 `map.pgm` / `map.yaml`；原图不覆盖。三维点云清洗仍在 GLIM map_editor 中完成。
- 前进/后退定距及旋转实验使用实时 `/odom` 和 20Hz 有加速度限制的闭环控制；在**真实运动安全准入**之前按钮不会驱动车辆，输出默认送 `/agt/studio/cmd_vel` 而非底盘 `/cmd_vel`。

## 运动安全准入（必须接入现场硬件安全链）

操作者确认实体急停、遥控接管及试验区域安全只是一个必要条件；仍必须有连续新鲜的 `/agt/safety/motion_permitted` 和 `/agt/safety/obstacle_clear` (`std_msgs/Bool`) **真值**、新鲜 `/odom`、人工点击“使能测试”。这两个主题必须由已经实车验收的控制/碰撞保护节点发布，**不要通过命令行强制发布 true 绕过准入**。若缺失或超时，fail-closed，停止下发非零速度。默认速度输出主题不与实际底盘连接，需经已经验收的 Motion Guard 连接。

运动期间至少现场有一人控制物理急停；驱动必须具备独立的心跳超时停车，防止电脑断电后继续行驶。开机无自动运动。若你只有旧 workbench 的安全服务，需在第二阶段实现安全状态 ROS Topic bridge，严禁跳过原来的门禁。

## 验收范围 / 明天顺序
1. 先 `--demo` 检查 UI 画面，无 ROS、不运动。
2. 实机只读连接，确认 `/odom`、GLIM 实际 `PointCloud2` 话题；录包、保存地图。
3. 在独立终端配置启动 GLIM 或将 `glim_config_path` 改为你的绝对路径后点击“启动 GLIM”；避免重复启动 Livox 驱动/TF 发布者。
4. 运动：先软件假反馈测试；之后检查 CAN、驱动 watchdog、实体急停、遥控接管、障碍保护、相同 TF 所有权，才允许低速 0.5 m 实车。
5. 不宣称本次已验证 Bunker/FW-mini 定距精度、时间同步、GLIM 优化结果、TF 或实车停止距离。

## 开发验证

```bash
PYTHONPATH=. python3 -m pytest -q test/test_core.py
python3 -m compileall -q agt_robot_studio
```
