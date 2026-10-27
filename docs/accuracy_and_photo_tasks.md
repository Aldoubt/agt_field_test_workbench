# 5/7/10 米精度测试与多点云台拍照

这是两个独立任务。精度测试固定同一起点、同一组终点，各测 5 次；多点拍照按记录的地点和云台角度顺序执行。复用 `agt_navigation_runtime/mission_runtime`，不再启动一套底盘、定位或 Nav2。

如果直接在地图上选多个点，使用新增的 [RViz 多点拍照入口](rviz_photo_tasks.md)，无需逐点把车辆开过去记录。

`scripts/field_tasks.py` 的 teach/accuracy/photos 只读取位姿或生成文件；新增 run/home 命令才显式发送距离任务。也可以继续用下面的 `ros2 action send_goal`。组合启动入口和短命令见 [启动模式](launch_modes.md)。当前无实车精度/多点拍照结果，不能把文件生成测试当成实车验收。

## 公共准备

先按 `cli_rviz_field_trial.md` 完成 start、map、relocalize、status、nav。确认定位有效、Nav2 active，结束其他导航目标，任务期间不要同时在 RViz 下发目标。

以下命令终端先执行：

```bash
cd ~/agt_field_test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

现有 `field_trial start` 不启动相机和巡检执行器。另开终端启动执行器并保持运行（已有 `/agt/mission/execute` 服务端时复用，不重复启动）：

```bash
ros2 run agt_navigation_runtime mission_runtime --ros-args \
  --params-file "$(ros2 pkg prefix agt_navigation_runtime)/share/agt_navigation_runtime/config/runtime.yaml" \
  -p record_root:="$PWD/data/captures/missions"
```

这个节点使用既有 `/navigate_to_pose` 和 `/camera_gimbal/acquire_view`；不控制底盘驱动的启停。精度任务没有拍照视图，因此不需要接相机。

取消整个巡检任务：

```bash
ros2 service call /agt/task/cancel std_srvs/srv/Trigger '{}'
```

等待任务终止并确认车辆停稳。`./field_trial cancel` 只取消 Nav2 子目标；运行巡检任务时优先用上面的父任务取消入口。暂停只在阶段边界生效，不作为急停。

## 1. 5/7/10 米精度测试

选择可容纳 10 米测试线和返程的场地，在地上标好起点和车体参考点。车头对准测试方向，停稳后生成本批任务（自动读取当前有效定位作为起点）：

```bash
python3 src/agt_field_test_workbench/scripts/field_tasks.py accuracy \
  --trials 5 --output missions/accuracy_5710
```

accuracy 不读取旧点位本中的 home，也不使用 --points；它从当前地图定位读取一次起点，保存到输出目录的 `start_pose.yaml`。同一批所有距离和返程都复用此起点。每次 accuracy 默认创建唯一新目录，生成成功后自动选为当前批次，run/home 默认使用该批次。也可用 --output 自定义新目录，或用 --directory 显式执行历史批次。未选批次不会回退到旧 missions/distance；已有任务输出不会被覆盖。

生成内容：

- `accuracy_5m.yaml`、`accuracy_7m.yaml`、`accuracy_10m.yaml`：先到固定 home，再到对应固定目标，停在那里。
- `return_home.yaml`：单独返程。
- `physical_measurements.csv`：15 条空白测量记录，未填入任何测试结果。

目标沿记录时的车头方向计算，坐标为 `x0+d*cos(yaw0), y0+d*sin(yaw0)`。**后续每轮使用同一份文件，不从每次实际停车位置重新向前加距离。** 在 RViz 核对路线和目标，并用卷尺/测距仪沿现场测试线标出真实的 5、7、10 米参考位置。Nav2 遇障会绕路，这里的米数是起终点距离，不是累计行驶里程。

以 5 米第 1 次为例，下面命令会开始行驶：

```bash
ros2 action send_goal /agt/mission/execute \
  agt_robot_interfaces/action/ExecuteInspectionMission \
  "{mission_file: '$PWD/missions/accuracy_5710/accuracy_5m.yaml', resume: false}" --feedback
```

等待最终 `success: true` 且车辆停稳，测量车体同一参考点相对地面目标的纵向、横向误差，填入 CSV，并保留现场照片。测量完成、人员离开车辆行驶范围后再返程：

```bash
ros2 action send_goal /agt/mission/execute \
  agt_robot_interfaces/action/ExecuteInspectionMission \
  "{mission_file: '$PWD/missions/accuracy_5710/return_home.yaml', resume: false}" --feedback
```

上述“出发 → 测量 → 返程”重复 5 次，然后把任务文件换成 `accuracy_7m.yaml`、`accuracy_10m.yaml`，各测 5 次。每轮开始前检查实际 home 偏差；偏差明显时人工复位到同一地面标记，保持原目标文件不变。返程也通过 Nav2 规划，可能转向再前进，不强制倒车。

每档记录成功次数/总次数、纵向误差、横向误差、位置误差 `sqrt(纵向误差²+横向误差²)`、朝向误差（能测则记），统计平均值与最大值；失败也保留，不只统计成功轮次。车载 TF/odom 用于诊断与复现，不能代替独立尺量。

若按现有项目指标对照，5/7/10 米平均位置误差分别为 12/25/35 厘米；可以先只记录实测结果，不让报表阈值阻止测试。真实碰撞、定位有效性和数据时效检查保持有效。

可在独立终端运行 `./field_trial record` 保存导航诊断 bag，结束后 Ctrl+C；图片与尺量记录另存。

## 2. 云台相机单点检查

2026-10-25 首次检查时串口尚未连接，用户接线后已检测到 Wasintek 相机 `/dev/video0` 和云台 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，图像与反馈实测在线。设备改变后重新确认设备名，不要把视频设备当成云台串口。

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/serial/by-id/
```

已读取相机支持格式：MJPEG 支持 1920×1080、30 fps。先用这个格式验证链路；默认 4K 更消耗解码和传输资源。若设备有变化，重新用 `v4l2-ctl -d /dev/video0 --list-formats-ext` 核对。

连接后，另开终端启动相机与云台（串口例子 `/dev/ttyUSB0` 必须换成实际设备，推荐稳定的 by-id 路径）：

```bash
ros2 launch autolabor_c1_bringup autolabor_c1.launch.py \
  device_path:="$(readlink -f /dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0)" \
  port_name:=/dev/ttyUSB0 \
  image_width:=1920 image_height:=1080 fps:=30.0 pixel_format:=mjpeg2rgb \
  gui:=false capture_output_root:="$PWD/data/captures/camera"
```

检查健康：

```bash
ros2 topic echo /camera_gimbal/health --once
```

应为 `state: 1`，camera_alive、gimbal_serial_connected、gimbal_feedback_alive、move_action_ready 都为 true。RViz 添加 Image，话题 `/cv_camera0/image_raw` 查看构图。

车辆停稳后单张试拍（会转动云台，heading/roll/pitch 单位为度）：

```bash
ros2 action send_goal /camera_gimbal/acquire_view \
  camera_gimbal_interfaces/action/AcquireView \
  '{heading: 0.0, roll: 0.0, pitch: 0.0, tolerance: 1.5, timeout: 8.0, stable_samples: 3, settle_time: 0.5, image_timeout: 2.0, save_image: true, tag: static_test, capture_name: static_test}' \
  --feedback
```

结果应为 `success: true`、`error_code: 0`，检查返回的 image_path 实际图片。此接口等待云台稳定后再取新图像；不是只保存任意一帧缓存图像。串口没有接通时先处理连接，不把失败当成拍照成功。

## 3. 指定多点依次拍照

用遥控器或单个 Nav2 目标把车移到希望拍照的位置，停稳、确认构图后记录。以下为三个点示例；角度需按现场构图调整，0 度是否正对目标以实际图像为准：

```bash
python3 src/agt_field_test_workbench/scripts/field_tasks.py teach P01 --heading 0 --pitch 0
# 移到第二点、停稳后执行
python3 src/agt_field_test_workbench/scripts/field_tasks.py teach P02 --heading 30 --pitch -10
# 移到第三点、停稳后执行
python3 src/agt_field_test_workbench/scripts/field_tasks.py teach P03 --heading -30 --pitch 0

python3 src/agt_field_test_workbench/scripts/field_tasks.py photos P01 P02 P03 \
  --output missions/photo_route.yaml
```

teach 只记录当前车体位姿与指定角度，不转动云台。可先用上一节的单张试拍调整并确定每个点的角度。现有任务 YAML 支持每点多个 views，可在同一点增加前/左/右等角度；生成器默认每点一张。

核对 `missions/photo_route.yaml` 中点位、顺序和角度后，开始任务：

```bash
ros2 action send_goal /agt/mission/execute \
  agt_robot_interfaces/action/ExecuteInspectionMission \
  "{mission_file: '$PWD/missions/photo_route.yaml', resume: false}" --feedback
```

流程：P01 导航 → 实测停稳 → 云台到位 → 新图拍照 → P02 → P03。默认停在最后一点，不自动返程。必需照片失败会结束该任务并记录原因，不假装完成后跳过。

执行器在 `data/captures/missions/<任务名_时间>/` 保存：

- `mission.yaml`：本次路线快照。
- `images/<点位>/`：逐点图片。
- `captures.csv` / `captures.jsonl`：图片时间、拍摄时机器人位姿及有效性、实际云台角度、错误码。
- `events.jsonl` / `manifest.json`：执行事件和最终状态。

最终应为 `success: true`、`completed_points: 3`，并实际查看三张图。可对返回目录执行：

```bash
ros2 run agt_navigation_runtime validate_records /实际返回的任务记录目录 \
  --expected-points 3 --views-per-point 1
```

这不要求 RTK，也不要求毫米级标定。多点相机实测通过后，再按图像用途调整分辨率、曝光和视角。

## 单测 5 m 的误差诊断记录

停稳、车头对准测试线后，创建一个新的批次目录：

```bash
./field_distance accuracy --distances 5 --trials 5 --output missions/accuracy_5m_batch1
```

另一个终端在发车前启动 `./field_trial record`，回程停稳后再用 Ctrl+C 结束录包。它会打印 `bags/field_trial_...` 输出路径；录包命令本身不会下发行车任务。每轮按 `run 5 --directory missions/accuracy_5m_batch1`、停稳尺量、`home --directory missions/accuracy_5m_batch1` 的顺序执行。

录包入口包含以下诊断链路（未发布的话题不会产生数据）：

| 目的 | 话题 |
| --- | --- |
| 定位与全局修正 | `/tf`、`/tf_static`、`/agt/odometry/local`、`/fastlio2/lio_odom`、`/agt/odometry/adapter_status`、`/agt/localization/status`、`/agt/localization/metrics`、`/agt/map_tracking/pose`、`/agt/map_tracking/status` |
| 指令到实际运动 | `/cmd_vel_nav`、`/cmd_vel`、`/mux/cmd_vel`、`/wheel/odom`、`/bunker_status` |
| 规划与避障 | `/map`、`/plan`、`/local_plan`、`/lookahead_point`、局部及全局 costmap/costmap_updates、`/local_costmap/published_footprint` |
| 任务阶段与恢复 | `/agt/mission/status`、`/agt/task/status`、navigate_to_pose / follow_path / backup 的 `_action/feedback` 和 `_action/status`、`/rosout`、`/parameter_events` |
| 定位复盘原始输入 | `/livox/lidar`、`/livox/imu` |

同时保留批次目录（尤其是 `start_pose.yaml`、任务 YAML 和填写后的 `physical_measurements.csv`）、`data/captures/missions/` 对应任务记录、`runs/` 对应启动配置及日志。bag 的 action feedback/status 不含完整原始目标请求，不能替代任务 YAML；parameter_events 也不能替代启动参数快照。

2 cm 是定位坐标内的到点门限。每轮独立记录车体同一参考点的纵向、横向、朝向误差和停车后位姿变化，并注明尺量时间、试次和 bag。先比较 map 与 odom 变化区分全局修正与局部漂移，再比较规划路径、控制指令、平滑后指令、guard 输出和实测速度，定位跟踪偏差、限幅延迟、打滑或制动距离。车轮里程计和 LIO 都不是独立精度真值。
