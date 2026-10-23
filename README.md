# AGT Field Test Workbench

集成仓库固定 15 个源码依赖 SHA，提供单一 Ubuntu 22.04 / ROS 2 Humble 工作区、受控建图和录包、标定版本、2D 地图编辑和本地网页入口。云端 Debian 13 宿主机中的 Ubuntu 22.04 / ROS 2 Humble Docker 容器已完成 57 个目标包的构建、114 项测试及两车无底盘启动检查；实机、CAN 和 bag 仍须现场验收。

## 部署

```bash
mkdir -p ~/agt_field_test_ws/src
git clone https://github.com/Aldoubt/agt_field_test_workbench.git \
  ~/agt_field_test_ws/src/agt_field_test_workbench
cd ~/agt_field_test_ws/src/agt_field_test_workbench
bash scripts/init_workspace.sh
# 在 Ubuntu 22.04 + ROS 2 Humble 工控机安装构建工具和上游清单未正确映射的系统依赖：
sudo apt-get install -y python3-rosdep python3-colcon-common-extensions \
  ros-humble-gtsam ros-humble-sophus libyaml-cpp-dev libeigen3-dev libasio-dev
# 首次使用 rosdep 时先运行：sudo rosdep init
rosdep update --rosdistro humble
bash scripts/build_humble.sh
bash scripts/run_workbench.sh
```

浏览器打开工控机本地 `http://127.0.0.1:8765`。远程访问使用 SSH 本地端口转发：`ssh -L 8765:127.0.0.1:8765 <工控机>`。网页只绑定本地回环接口，没有任意命令或速度发布接口。

统一底盘启动（**默认不启动底盘驱动**）：

```bash
bash scripts/run_field_test.sh robot:=bunker
bash scripts/run_field_test.sh robot:=fwmini
# 若明确完成现场安全检查，单独启动底盘 CAN 路径：
bash scripts/run_field_test.sh robot:=bunker enable_chassis:=true
bash scripts/run_field_test.sh robot:=fwmini enable_chassis:=true \
  fwmini_geometry_file:=/absolute/path/to/measured_geometry.yaml
# 已完成 Bunker 地图/定位/安全验收后，才显式打开 Nav2：
bash scripts/run_field_test.sh robot:=bunker enable_chassis:=true \
  enable_localization:=true enable_navigation:=true \
  global_map:=/absolute/path/to/map_package/map.pcd \
  map_yaml:=/absolute/path/to/maps/versions/<id>/map.yaml
```

同等 ROS 入口：`ros2 launch agt_field_test_bringup field_test.launch.py robot:=bunker|fwmini`。建议使用 `run_field_test.sh`，它用 `flock` 阻止两次脚本启动竞争 CAN。`Ctrl+C` 停止。FW-mini 的底盘适配器启动后仍需显式调用 `/fwmini/enable_motion`；当前未做过现场安全验收，不应启用运动。
Bunker 和 FW-mini 的底盘单独启动时，入口只接入低速手动 command guard（命令超时 0.20 s），不依赖全局定位；FW-mini 适配器仍默认禁止运动并执行自己的超时。显式 `enable_navigation:=true` 时由 V3 导航入口启动其 guard，并校验 `ACCEPTED` 地图与定位 PCD 的 SHA-256 对应关系。

## 当前闭环

- 审计：`python3 scripts/audit.py` 报告实际 SHA、dirty 状态、重复 ROS 包名；`.repos` 固定版本。建图源码上的两处受控集成补丁由 `apply_mapping_calibration.py` 精确校验，未修改其 Git HEAD。
- 建图：网页调用原 `run_mid360_mapping.sh`。bag 回放在隔离 Domain 89 运行 FAST-LIO2 + PGO；实机传感器模式需要对应机器人已核实的 MID360 IP JSON，调用原脚本并由它录制一次 raw bag。成功后复用 `agt_pcd2grid_exporter` 自动生成 PGM/YAML，作为 `DRAFT` 进入 Map Studio。输入 bag 不改动。取消仅向本次启动的进程组发信号。
- 标定：候选数据和 SHA-256 存于 `calibration/<robot>/`；已验证状态需要 `runs/` 或 `reports/` 中的 JSON 报告，且 `checks.repeatability/motion/geometry` 均为 `PASS`。映射时把选定 `r_il/t_il` 写入独立 FAST-LIO2 参数文件并传给原 launch。时间偏差与底盘安装变换只记录，尚未接入算法和 URDF；现场需要另行核验。
- 录包：五种模板，仅录取当前存在的话题；启动前检查剩余磁盘，保存缺失话题、类型、代码 SHA、底盘与标定信息。停止时检查 `metadata.yaml`、消息数和存储文件；异常标记 `INCOMPLETE`。
- Map Studio：从本工作区产物导入 PGM/YAML，缩放、平移、矩形擦除、障碍线、禁行 polygon、撤销/重做，关联源 PCD 时显示 3D 点云采样。保存生成不可覆盖的新版本和 `keepout_zones.yaml`，可在浏览器下载 ZIP，保留来源和 SHA-256。审核状态不阻断编辑。
- 运行状态：网页读取当前 ROS 话题，并显示缺失项；无 ROS 时明确显示 `ROS_UNAVAILABLE`。Navigation 页面只显示状态，不提供未经验证的运动按钮。

## 已核实的冲突和限制

导航仓库依赖锁分别使用 Livox driver `4a1def9`、FAST-LIO2 `f516daa`、Batch-LIO `fe4a9cb`；建图仓库使用 Livox `13eb05e`、FAST-LIO2/PGO fork `7f664a6`、Batch-LIO fork `7ceb276`。统一 `.repos` 选择建图 Livox/FAST-LIO2 和导航 Batch-LIO，每个 ROS 包仅有一份源码。V3 的 FAST-LIO2 导航适配器与此 fork 的组合已通过 Humble 编译，仍需 rosbag 运行验证；默认导航配置保持 Batch-LIO。V3 原 `hardware.launch.py` 引用 `bunker_base`，而现有底盘仓库的包名是 `agt_bunker_base`；新入口直接启动已编译的 `agt_bunker_base`，不调用该旧硬件入口。

FW-mini 厂家 `yhs_can_control` 与 `yhs_can_interfaces` 位于其独立仓库，`package.xml` 许可仍为 TODO；集成仓库没有复制厂家源码。FW-mini 的 Humble 软件包已编译，CAN 反馈、底盘运动和 MID360 安装 TF 均未验收，因此其定位路径在新 launch 中阻断；基础描述和 sensor-only 建图入口可用。V3 的持续校正已具有 tracker → LocalizationManager 的候选应用路径和门控，新 launch 在启用定位时默认开 `enable_map_tracking`，但还没有本次实测的 A/B rosbag 结果或结构化逐候选日志证据。

完整审计、验证状态、周六现场清单见 [`docs/field_delivery.md`](docs/field_delivery.md)。
