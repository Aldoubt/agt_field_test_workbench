# 17:35 多点拍照任务：第二段导航中断和云台回中修复

本次任务 P001 的左、中、右三张照片确实已保存；P002 未到达。定位跟踪失效与云台没有回中是两个独立问题。源码已修复并构建，现场旧进程尚未重载，没有执行现场导航、云台目标、清图或底盘启停。

## 现场证据

任务 `rviz_patrol_20261025_173528_20261025_173528`：17:36:27 到达 P001，17:36:30/32/35 分别完成三张图。实际 heading −29.33/−1.68/+29.04°。最后一张记录后约 1 ms 即发送 P002，原实现没有拍照后的回中步骤。

17:36:59–17:37:05，跟踪 attempt 76–79 均 GICP 收敛，但 overlap 0.381794/0.375434/0.360235/0.365697，连续四次低于 0.40。fitness 0.315–0.367，未触发 fitness 门槛。状态 HOLD → DEGRADED → RECOVERY_REQUIRED，管理器在 17:37:05.748 撤销全局修正并请求重定位，停止 map→odom 更新。Nav2 随后报 TF future extrapolation / Controller patience exceeded。自动候选引导 BBS 在约 17 秒后失败。

接收探测仍见本地 odom 10.2 Hz，戳年龄 P50 35.8 / P95 53.91 / 最大 103.9 ms；local_odom_fresh=true。Nav2 两节点 active，但 global_correction_valid=false、waiting_global_pose，因此当前不能继续导航。最新底盘 vehicle_state=2、error_code=4；当前 SDK 的 SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK=0x0004，需恢复遥控连接。这是最新状态，不作为之前第二段导航失败的起因。

照片的 pose_valid 均为 false。接收诊断测得相机时间戳领先 ROS 时间 1.04–1.12 秒，运行器当时查图像时刻 TF 报未来外推。不能将“图像落盘”称为“位姿同步记录通过”。旧相机进程始于 16:55，早于当前 17:33 的时钟保持会话；先重载相机后复核，未改写图像时间戳。

## 修改

- MissionRuntime 新增可选 recenter_after_capture，默认 false；field_photo runtime 显式启用，并在 start 时读取实际参数确认加载。
- 三视图结束后，先检查底盘仍静止，再调用 MovePantilt 到 heading/roll/pitch=0，容差 3°、连续两次稳定编码器反馈、动作预算 8 秒。成功、结果码和最终角度均符合才计完成该点、发送下一点。失败、超时、取消遵循已有有界取消及停止确认。没有第四张照片，没有底盘回正目标；无视图的 RETURN_HOME 不额外回中。
- MapTracker 原查询为 0.5–30 m 扫描，目标地图为以预测位姿为中心的 map 坐标圆柱（半径 20 m、半高 5 m）。现在查询也按同一 map 坐标空间范围裁剪，再交给原生 GICP；不按匹配结果或地图内已有点筛选查询。
- 40% overlap、fitness、最小 1500 查询点、创新门槛、里程计 0.20 秒时效、TF 所有权和全局重定位算法保留。裁剪后不足 1500 点仍拒绝。

## 离线验证和边界

- 两包构建通过：agt_navigation_runtime、agt_map_tracker。
- 拍照流程 22 项通过：多点、返航、失败停止，以及新增回中成功/失败/错误角度/超时/取消，验证等待回中期间没有下一导航目标。
- 查询范围与补丁 7 项通过：重力方向、保留空间内不匹配点、裁剪后点数不足、低 overlap、大创新仍拒绝；干净锁定克隆应用、重复应用、校验及冲突保留。
- 扩展回归为 120 通过、2 失败。失败来自原 test_lio_modes 的两个 launch 夹具缺少既有 SC 补丁新增的 global_backend/sc_database 上下文参数；本轮未修改 LIO 切换实现或该测试。未把全量结果标为通过。
- 四组故障区域附近的历史建图关键帧（944/963/979/1000，三帧运动补偿累积）离线 A/B，种子平移扰动 5 cm/2 cm、yaw 扰动 0.2°：

| 关键帧 | 修改前 overlap | 同范围 overlap | 修改前/后与建图位姿的平移差 |
|---|---:|---:|---:|
| 944 | 0.5706 | 0.9985 | 4.40 / 1.74 cm |
| 963 | 0.5626 | 0.9980 | 2.03 / 0.91 cm |
| 979 | 0.5693 | 0.9955 | 4.51 / 2.97 cm |
| 1000 | 0.5567 | 0.9948 | 2.69 / 0.83 cm |

这些是建图数据自一致性结果，没有外部真值，也没有本次失败时的原始扫描，不能认定实车该路段已经通过。当前停车位置接收的单帧全部位于目标空间内，A/B 相同，未复现故障区域低 overlap。

## 加载与继续测试（由操作者执行）

不用重启电脑。先保持停车并恢复遥控连接；不要在旧任务仍执行时重载。只需要重载跟踪节点、相机和拍照执行器，主导航/底盘进程可以保持。

1. 在原 `./field_photo runtime` 和 `./field_photo camera` 终端分别 Ctrl+C。不要关闭 field_trial 主终端的时钟保持会话。
2. 新终端执行以下代码，仅重载 map_tracker；单一进程检查防止误杀其它节点。此命令不启动或停止底盘：

```bash
cd ~/agt_field_test_ws
bash -c '
set -euo pipefail
tracker_pid="$(pgrep -f "^/usr/bin/python3 /home/yangxuan/agt_field_test_ws/install/agt_map_tracker/lib/agt_map_tracker/map_tracker( |$)" || true)"
if [[ ! "$tracker_pid" =~ ^[0-9]+$ ]]; then
  echo "未找到唯一旧 map_tracker；先检查 ros2 node list 和进程，未执行重载。"
  exit 1
fi
kill -INT "$tracker_pid"
for attempt in {1..50}; do
  if ! kill -0 "$tracker_pid" 2>/dev/null; then break; fi
  sleep 0.1
done
if kill -0 "$tracker_pid" 2>/dev/null; then
  echo "旧节点尚未退出；未启动第二个 tracker。"
  exit 1
fi
exec bash src/agt_field_test_workbench/scripts/ros_exec.sh ros2 launch agt_map_tracker map_tracker.launch.py global_map:=/home/yangxuan/agt_field_test_ws/maps/clouds/20261024T124930Z-714128a2/cloud.pcd
'
```

3. 另外两个终端各执行一个命令，保持运行：

```bash
cd ~/agt_field_test_ws
./field_photo camera
```

```bash
cd ~/agt_field_test_ws
./field_photo runtime
```

4. 机器人静止后请求重定位并检查状态：

```bash
cd ~/agt_field_test_ws
./field_trial relocalize --timeout-sec 30
./field_trial status --seconds 5
```

Trigger success 只说明请求已提交；必须等待 global_correction_valid=true、local_odom_fresh=true、底盘错误消失。若重定位仍失败，停止在此，不重复发送任务。Nav2 如已 active 不需再 Startup；确实未 active 时执行 `./field_trial nav`。

5. 在 RViz 检查定位正确，再按需要 clear/重新排队/start；start 会发送实车运动任务，由操作者确定环境和遥控模式后执行：

```bash
./field_photo clear
# 在 RViz 添加测试点
./field_photo start
```

首次建议只选两个较近点，观察三张图 → 云台回中 → 再行驶。用 /agt/map_tracking/status 检查 query_points_before_map_crop/query_points、overlap、reason_codes，并检查新照片 pose_valid。相机重载后若仍超前，需要继续排查相机时钟，不改戳绕过。

## 补丁和证据

- `src/agt_field_test_workbench/patches/photo_mission.patch` 与 `photo_mission_manifest.json`：运行器、steady 等待器、MovePantilt 依赖。
- `map_tracking_support.patch` 与对应 manifest：查询空间范围修正。
- init_workspace.sh、build_humble.sh、audit.py 已接入；锁定导航 HEAD 保持 b3a60487dff973c6326aca2ca2e9db1e78daeaaa。audit 无错 SHA、无重复 ROS 包，导航修改识别为 EXPECTED；原有 bunker_ros2/fast_lio2_mapping 脏修改保留。
- 原源码、补丁、集成脚本备份在 `runs/photo_recenter_20261025/before/`；现场 manifest/events 副本、故障跟踪记录、接收诊断、离线 A/B 和测试 XML 均在同目录。
