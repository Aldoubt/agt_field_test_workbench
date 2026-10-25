# AGT Robot Studio — ROS 2 Humble 桌面工作台（v0.2.0）

面向日常使用的五个页面：**建图、地图、定位与导航、参数、日志**。操作不需要打开终端。Studio 复用现有 Field Test Workbench、AGT Navigation V3、GLIM 和 Nav2，不创建第二套运动或定位后端。

顶部全局状态栏在所有页面持续显示当前设备、会话／定位状态、底盘反馈通讯，以及雷达、IMU 和 `base_link` 里程计的实际到达频率。底盘“通讯在线”只表示反馈新鲜，不等于获得运动许可；传感器没有新数据时显示离线，不使用标称频率伪装在线。

## 操作员流程

1. **建图**：首屏选择已加载车辆，查看 MID360、IMU 和 `base_link` 里程计的实际到达频率。可分别连接传感器和底盘，也可填写地图名称、选择 GLIM 配置后一键“开始建图”。名称写入本次 `session.json`，并随 PCD 和二维地图 lineage 保留。“保存建图数据”开启时保留独立 `glim_dump`、`raw_bag` 和 `session.json`；关闭时只保留配置快照。“开始录制”用于不建图时的独立录包。“关闭所有 ROS 节点”仅关闭 Studio 与 Workbench 管理的 ROS 进程，不是硬件急停。
2. **地图**：用“编辑三维地图”清理三维残影并导出 PCD；点击“导入 PCD”和“生成二维地图”。只需要清理导航栅格时，直接用“空闲”或“未知”画笔覆盖人的残影和鬼影。选择“电子围栏（多边形）”后逐点点击，双击或按“完成围栏”；保存时后端会再次把围栏烘焙成 PGM 障碍，并保存 `keepout_zones.yaml` 与 `keepout_bake.yaml`。地图选择框显示“名称＋短版本＋状态”，底层仍使用不可变版本 ID；旧地图没有名称时显示原 ID，不补造名称。可撤销当前编辑并保存草稿或导航地图版本。
3. **定位与导航**：按名称选择带同源 PCD 的地图，并通过明确标注的“里程计后端”“重定位后端”下拉框选择 Batch-LIO／FAST-LIO2 和 BBS／KISS。两个里程计后端互斥，统一输出 `/agt/odometry/local` 的 `base_link` 里程计；FAST-LIO2 由现有 `agt_fastlio_adapter` 转换。默认重定位是 3D-BBS＋GICP；KISS-Matcher＋GICP 是实验选项。地图资产状态会区分 BBS 体素资产与 AGT Polar Context，后者不是 Scan Context（SC）。定位有效后可直接点击地图目标，或输入距离选择向前／向后。切换到“添加航点（可拍照）”后逐点点击地图，设置云台航向／俯仰，保存路线再执行：V3 `mission_runtime` 顺序完成 NavigateToPose、停稳确认、AcquireView、图像与位姿记录，然后进入下一点。必需视角拍照失败会停止任务，不会假装成功。
4. **参数**：配置显示话题、GLIM 与输出目录、录包模板与自定义话题、三转二参数以及 C1 设备路径。点击“保存参数”后写入 `~/.config/agt_robot_studio/studio.yaml`，下次启动自动加载；设备参数在下次会话生效，显示话题在重连 ROS 显示适配器后生效。
5. **日志**：查看并保存本次操作记录。

已完成的标定和现场证据通过 `studio.yaml` 中的 `calibration_bundle`、`mount_profile`、`safety_report` 等配置复用，不要求每次重新标定。后端已有的地图、标定和运行合同仍然有效；界面不会伪造验收报告或许可状态。

## 地图编辑与严格导航校验

- 二维画笔和电子围栏只改变导航占用栅格；不可变版本仍保留原 `source_pcd`、`source_pcd_sha256` 和同源重定位资产关联。若三维残影已经影响定位，应先在 GLIM Map Editor 清理并导出新 PCD，再重新生成二维地图，并基于该新 PCD 重建同源重定位资产，不能把旧 PCD 的描述子库当作新地图资产。
- 每次编辑都保存为新的不可变 `map_id`，并产生与新 PGM 对应的 `map_sha256`；父版本不被覆盖。电子围栏由后端再次烘焙进 `map.pgm`，Nav2 当前直接读取该静态地图，不依赖仅存在于界面中的红色覆盖层。
- 推荐先保存为 **DRAFT**，选择这个新版本启动定位会话，检查全局重定位、地图方向、Origin 和围栏位置。DRAFT 可用于定位验证，但不能直接启动正式导航。
- 验证后再保存／选择 **ACCEPTED** 导航版本。正式导航仍要求 VERIFIED 标定，以及与当前地图 SHA、运动参数配置 SHA 和标定 SHA 全部匹配的 `PHYSICAL_NAVIGATION` 现场报告；编辑地图后，旧报告不会因旧地图曾经 ACCEPTED 而自动复用。
- 路线同时绑定 `map_id`、地图 PGM SHA 和 YAML SHA。地图编辑后应在同一新版本中重新保存路线；选择不同版本或仍运行旧地图会话时，后端会拒绝目标，而不是悄悄在旧地图上运动。
- 电子围栏是静态全局规划禁行约束，不是硬件急停、动态避障或安全认证围栏。保留这些校验是为了让导航证据和实际加载的地图一致，不应通过跳过检查来协调编辑能力。

## 配置

```yaml
robot: bunker
workbench_api: http://127.0.0.1:8765
workbench_root: ''             # 空值时自动识别仓库根目录
calibration_bundle: /absolute/path/to/verified_calibration.yaml
safety_report: /absolute/path/to/existing_navigation_report.json
glim_config_path: /absolute/path/to/glim/config
bag_root: ~/agt_studio_runs
record_template: full_field_test
global_backend: bbs            # bbs 或 kiss；默认不自动切换
lio_backend: batch_lio         # batch_lio 或 fastlio2
save_mapping_data: true
enable_camera_gimbal: true
camera_device_path: /dev/video0
gimbal_port_name: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
# 仅在部署配置中复用已确认的现场操作条件；程序不会默认填 true。
session_checks:
  wiring: true
  physical_estop: true
  remote_takeover: true
  clear_area: true
  supervised: true
```

录包不限于雷达和 IMU。独立录包模板包括 `calibration`、`mapping`、`localization`、`chassis_motion`、`full_field_test` 和 `motion_diagnostic`；在参数页填写自定义绝对话题后，独立录包改用该列表，建图录包也复用同一列表。独立录包只选择当前 ROS 中实际存在的话题，并记录实际话题、类型、QoS 和缺失话题；自定义列表留空时保持原模板行为。

两种粗定位后端的资产合同不同：candidate BBS 使用同源 `polar_context.db` 和 `voxelmaps_coords`，当前原生流程是 ring key 全库排序、`descriptor_prefilter=40`、sector 二次排序、`candidate_top_k=2`、局部 BBS，再由 small_gicp GICP 精化；没有描述子数据库的旧地图走整图 BBS 回退。KISS-Matcher 不读取这两类资产，而是从同源 PCD 做全图粗匹配，有 `global_map_downsampled.pcd` 时优先复用，之后同样使用 GICP。参数页展示这些真实默认值，但不把它们伪装成在线可调参数。

FW-mini 可同时设置 `mount_profile`、`geometry_file`；需要显式 Livox 配置时设置 `livox_config`。Workbench 未运行时，Studio 会在本机后台启动 `workbench.server`，用户不需要手动执行命令。首次打开时窗口会按屏幕可用区域与 DPI 调整尺寸、字体和侧栏，之后仍可手动拉伸；Qt 高 DPI 缩放在应用创建前启用。

## 构建与启动（部署人员）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_robot_studio --symlink-install
source install/setup.bash
ros2 run agt_robot_studio agt_robot_studio \
  --config ~/ros2_ws/src/agt_field_test_workbench/agt_robot_studio/config/studio.yaml
```

离线界面演示：

```bash
ros2 run agt_robot_studio agt_robot_studio --demo
```

## 架构边界

- `ros_worker.py` 只订阅点云、IMU、OccupancyGrid 和里程计用于显示与频率统计；没有 Twist Publisher、Action Client 或运动控制周期。
- `workbench_client.py` 在独立线程调用本机 Workbench API，避免网络／磁盘操作卡住 Qt。
- 地图版本继续使用 `workbench.maps`；三维 PCD、二维 PGM/YAML 和标定来源保持关联。
- 重定位使用 `/agt/localization/relocalize`；点击导航使用 `NavigateToPose`；相对距离使用现有 map-frame 位姿生成目标。
- 实体急停、最终速度保护、CAN 反馈和驱动 watchdog 仍属于现有后端与硬件，不由 Qt 取代。

## 测试

```bash
PYTHONPATH=. python3 -m pytest -q test
python3 -m compileall -q agt_robot_studio
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s test -p 'test_qt_smoke.py'
```

这些检查只覆盖源代码逻辑和离线 GUI。GLIM、Humble lifecycle、全局重定位、CAN 与实车导航仍以目标机运行结果为准。
