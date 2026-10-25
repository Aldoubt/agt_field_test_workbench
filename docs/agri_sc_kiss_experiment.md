# 农科院 Scan Context + KISS-Matcher 定位实验交付

本分支以远端 `feature/field-automation-20261024` 的
`7039bf7fdfba627b84e76be255b1a2c5e9a6ef48` 为基线，开发分支
`feature/agri-sc-kiss-20261025`。交付代码为 `IMPLEMENTED`，下列软件验证为
`AUTOMATED_TESTED`；`REAL_BAG_VERIFIED=NOT_RUN`、`HARDWARE_VERIFIED=NOT_RUN`。
没有农科院真实地图、bag、设备或现场监督，不能据此声称定位精度或导航实车验收通过。

## 代码范围与调用链

| 文件 | 必要性 |
|---|---|
| `agt_sc_kiss/{CMakeLists.txt,package.xml,NOTICE.md,src/bindings.cpp}` | 独立可选包；锁定并直接编译上游 C++ Scan Context；静态库 Top-K 薄封装 |
| `agt_sc_kiss/agt_sc_kiss/{geometry,database}.py` | 建库和在线查询共用坐标、标定、裁剪、降采样、重力校平；独立数据库及哈希 |
| `agt_sc_kiss/agt_sc_kiss/{localizer,runtime}.py`、`scripts/sc_kiss_localizer` | 真正调用已构建 KISS 粗配准和原生 GICP，适配现有运行节点 |
| `dependencies/{field_test.repos,scancontext.json}` | 新增 SC 固定提交和五个源文件哈希，原依赖版本不变 |
| `patches/sc_kiss_{runtime.patch,manifest.json}`、`scripts/apply_sc_kiss_integration.py` | 在原控制补丁后给导航依赖应用三文件增量；未知本地修改直接保留并报错 |
| `scripts/{apply_control_integration.py,audit.py,init_workspace.sh,build_humble.sh}` | 识别组合补丁、防止重复应用；审计合并各补丁文件集合 |
| `scripts/build_kiss_optional.py` | 复用原锁定构建配方，仅支持显式工作区和源路径解析 |
| `scripts/{build_sc_experiment.sh,build_sc_database.py,sc_kiss_ab.py}` | 可选部署、最小建库、同查询离线 A/B 命令 |
| `workbench/{session.py,relocalization.py}` | 两种模式、DB/PCD/导航地图身份检查、同配置复用 |
| `agt_field_test_bringup/launch/field_test.launch.py` | 向系统定位启动链传递模式和 DB 路径 |
| `scripts/ros_bridge.py` | 采集真实后端状态 JSON，供 Qt 展示 |
| `agt_robot_studio/agt_robot_studio/ui.py`、`config/studio.yaml` | 两个真实选项、DB 参数、耗时/质量/失败原因及候选日志 |
| `tests/{test_sc_mode.py,sc_kiss/*}`、Studio 对应测试 | 补丁保留性、地图身份、真实算法、ROS 运行链和 GUI 请求载荷 |

导航依赖增量只改：

- `bringup/agt_system_bringup/launch/localization.launch.py`。
- `navigation/localization/agt_global_relocalization/launch/global_relocalization.launch.py`。
- `navigation/localization/agt_global_relocalization/agt_global_relocalization/global_relocalization.py`。

真实链路为：Qt 请求 → Sessions → field launch → system localization launch →
原 GlobalRelocalization 节点的 `global_backend` 参数。
默认 `bbs` 保持原 Polar Context 候选 BBS / 无资产整图 BBS → 原 small_gicp。
手选 `sc_kiss` 才加载可选包：SC Top-K → KISS → **原有** `map_gicp_tracker` →
同一 `/agt/relocalization/pose` → **未修改的** Localization Manager。
没有第二套 Manager、控制者、TF 发布者或 Action 客户端。

SC 请求在现有节点的工作线程中执行，ROS 反馈仍继续处理。每次 SDK 调用拥有独立进程组，
超时、停止和运动取消会终止整组；失败不会发布有效位姿。Manager 仍唯一拥有 `map→odom`，
其原有时间对齐、协方差、定位许可与底盘门禁不变。显式请求重定位仍按原 Manager 行为清除定位许可；
失败不以错误结果改变校正。相机、路线、三视图、地图清理、FAST-LIO2 主体/雅可比/时间同步、底盘和 Nav2 未重写。

## 开源代码与许可

| 项目 | 固定版本 | 许可与复用方式 |
|---|---|---|
| gisbi-kim/scancontext | `93672835bb85e9c03fedf0fbaf2d0491df12161e` | CC BY-NC-SA 4.0；KAIST/NAVER LABS，Kim & Kim，IROS 2018；适配包同许可，见 `agt_sc_kiss/NOTICE.md` |
| MIT-SPARK/KISS-Matcher | `8b58b118e00c7a5a22448741f8d956d638a44ae2` | MIT；复用现有 `dependencies/kiss_python_build.json` 配方，实际编译 Python 扩展 |
| small_gicp | `2c6c7831bc1f5ada2ee5300027fe7d7aee66827d` | MIT；继续使用原导航包现有 C++ GICP 可执行程序 |
| 3D-BBS | `41529a34a2fb9618b5ff560fb3c2363f1615666d` | 保持原 CPU 构建及算法实现 |

SC 的描述子、ring key、距离和 yaw 对齐全部来自原实现；静态查询不使用在线闭环的“排除最近 50 帧”规则，
也不把查询插入数据库。仅移除未使用的 ROS1/OpenCV 头依赖，并明确固定 30 m 半径和 body 坐标下 0 m 高度偏置。
nanoflann/KD-tree 文件保留上游 BSD 署名；tictoc 保留原作者署名。该可选组件按本轮非商业科研范围使用。
主 BBS 运行路径无需导入或编译 SC/KISS。

## 坐标与接受条件

- 原始在线云必须是 `mount_lidar_frame`。通过**所选 LIO YAML 的实际正向** `T_body_lidar` 转为 `mapping_body`；不复制或猜测正负号。
- 建库关键帧必须是同一地图的 body 云、`T_map_body` 位姿。支持现有 `poses.txt` 和精确时间戳的 `poses_timed.txt`。
- 建库/查询共用传感器坐标量程 0.5–30 m、body 坐标 0.25 m first-point voxel；yaw 保留，roll/pitch 用 `Rz(-yaw) R_map_body` 去除。
- 在线 roll/pitch 来自扫描时间戳的 `odom→base_link` TF 与同一安装外参，不依赖人工初始位置或 yaw。
- SC 输出候选位置和 yaw；KISS 的 source 是 SC 旋转后的 body 查询，target 是候选位置周围世界点云减候选中心。
  最终 `R=R_kiss R_seed`、`t=center+t_kiss`，偏移只恢复一次；GICP 输入仍是原 body 查询。
- 发布 `T_map_base=T_map_body T_body_base`，安装偏移只转换一次。ROS 集成测试使用非零平移、非零安装倾角验证。
- SC distance ≤0.13、KISS 至少 20 最终内点，精配准必须收敛且非退化；继续满足原 score≥0.50、fitness≤1.00、overlap≥0.20。
  相近质量却相距超过 2 m 的候选拒绝；不放宽现有门控。
- DB 记录 PCD、导航 YAML/图像、关键帧/位姿和描述子身份，另校验完整元数据哈希。
  Sessions 先校验选定地图身份；运行适配层再次校验标定/预处理、PCD、导航资产，结果输出前再查地图身份。
  DB 目录不覆盖已有版本或 Polar Context。移动/修改导航资产后应对选定版本重新建库。

## 使用两种模式

在已有稳定 Humble 导航部署中增量安装：

```bash
export AGT_FIELD_TEST_WS="$HOME/agt_field_test_ws"
REPO="$AGT_FIELD_TEST_WS/src/agt_field_test_workbench"
bash "$REPO/scripts/init_workspace.sh"
bash "$REPO/scripts/build_sc_experiment.sh"
source "$AGT_FIELD_TEST_WS/install/setup.bash"
```

初始化只补缺少的固定依赖并应用已审阅补丁；未知修改报错保留。可选构建复用可导入的 KISS，
不存在才运行原锁定配方。需要系统 PCL、Eigen、TBB、Python 开发头和 pybind11；离线资源记录还需 GNU `time`。
该增量命令依赖原车辆导航部署已能构建，不替代完整车辆环境安装。

Qt 参数页填写独立 SC DB 目录；定位页选择“3D-BBS + GICP”（默认）或
“Scan Context + KISS-Matcher + GICP（实验）”。保留原启动/停止/重定位按钮。
切换前先停止定位/导航，再选择模式启动，保持传感器和底盘独立生命周期。
已有定位配置不一致时，后端会拒绝复用，不能只改显示文字。

已有 ROS 启动入口可用：

```bash
# 以下是已有传感器/描述/标定部署上的定位启动，不开启运动
ros2 launch agt_system_bringup localization.launch.py \
  lio_backend:=fastlio2 fastlio_config:=/data/agri/fastlio.yaml \
  global_map:=/data/agri/global_map.pcd \
  relocalization_assets:=/data/agri/bbs_assets global_backend:=bbs

# 实验入口；PCD 和标定与上面一致
ros2 launch agt_system_bringup localization.launch.py \
  lio_backend:=fastlio2 fastlio_config:=/data/agri/fastlio.yaml \
  global_map:=/data/agri/global_map.pcd \
  global_backend:=sc_kiss sc_database:=/data/agri/sc_db_v1

ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger '{}'
ros2 topic echo /agt/global_relocalization/status
```

状态含模式、成功与失败、总耗时、SC 候选分数、KISS 内点及 GICP Fitness/Overlap。
Qt 日志保留完整诊断，状态标签展示摘要。定位成功不直接发布速度。

## 最小建库命令

已有 no-PGO FAST-LIO 采集产物时，复用原精确时间戳重建方法：

```bash
python3 "$REPO/scripts/build_sc_database.py" \
  --artifact /data/agri/run/map_package \
  --map /data/agri/global_map.pcd --map-yaml /data/agri/map.yaml \
  --lio-config /data/agri/fastlio.yaml --output /data/agri/sc_db_v1
```

已有关键帧时用 `--keyframes /data/agri/keyframes` 替换 `--artifact ...`。
格式：`patches/000001.pcd` 为 body 云；`poses.txt` 每行
`000001.pcd x y z qw qx qy qz`，或 `poses_timed.txt` 每行多一个位于名称之后的时间戳。
数据库和重建目录均要求新路径，不覆盖旧资产；只有融合 PCD 而无关键帧/轨迹时明确拒绝建库。

若只有原始 bag，可先复用原离线建图入口生成 `map_package/{frames.csv,trajectory.csv,points.raw}`：

```bash
python3 "$REPO/scripts/no_pgo_mapping.py" --bag /data/agri/raw_bag \
  --config /data/agri/fastlio.yaml --output /data/agri/sc_mapping_run \
  --imu-offset 0.0 --lidar-offset 0.0
```

这里的时间偏移参数必须使用原验收配置值；示例 0.0 不代表替换既有同步修复。
随后按既有地图投影/验收流程取得对应导航 YAML，再对同一来源建库。该真实 bag 步骤本云端未运行。

## 一键离线 A/B

已编译并 source overlay 后：

```bash
python3 "$REPO/scripts/sc_kiss_ab.py" --synthetic --output /tmp/agri_ab_new
```

该命令生成三个不同地点的真实模拟关键帧和已知 SE(3) 查询，然后对**同一 PCD 文件**运行
原整图 BBS+GICP 与真 SC+KISS+GICP，记录 JSON、源码输入哈希、时间、CPU、RSS。
它没有 ROS 位姿发布或运动输出；`SYNTHETIC_KNOWN_SE3` 精度只表示合成变换，不是农科院精度。

实际查询使用 `--spec /data/agri/ab.json`，最小格式：

```json
{
  "query_frame": "mapping_body",
  "map": "/data/agri/global_map.pcd",
  "database": "/data/agri/sc_db_v1",
  "lio_config": "/data/agri/fastlio.yaml",
  "relocalization_assets": "/data/agri/bbs_assets",
  "timeout_sec": 30.0,
  "queries": [{"pcd": "/data/agri/query_body.pcd", "roll": 0.0, "pitch": 0.0}]
}
```

提供有效 Polar Context 资产时，BBS 列复用原候选 BBS 命令；无资产时测原整图回退。
roll/pitch 为该 body 查询的重力姿态弧度值，不能从错误初始定位猜测。
无独立真值时位置/航向误差与是否错误接受为 `NOT_RUN`；不以算法自己的结果充当真值。
有独立测量时可添加 `truth_kind: EXTERNAL_SURVEY`、`truth: {xyz: [...], yaw: ...}`、
`truth_evidence` 文件及其 `truth_evidence_sha256`；真值必须采用 `T_map_body` 坐标约定。
有真值时，离线错误接受标记采用位置误差 >0.5 m 或航向误差 >10°；它仅用于报告，不修改运行门控。
原 BBS 可执行程序未报告独立精配准时间，因此该列明确写 `NOT_REPORTED_BY_EXISTING_EXECUTABLE`。

实际本轮数值表见 [合成 A/B 实测](evidence/agri_sc_kiss_20261025/ab_results.md)。
阶段耗时属于最终选中候选，总耗时包含所有候选及进程开销。
实际本轮结果见 [证据目录](evidence/agri_sc_kiss_20261025/) 和 [合成 A/B JSON](evidence/agri_sc_kiss_20261025/comparison.json)。
整图 BBS 在宽范围倾斜合成场景 30 秒内未完成；未改变生产参数以掩盖它。
另测原 Polar Context+BBS 对同一倾斜场景的原 Fitness 门控拒绝，以及水平查询的成功接受。
这不构成任一算法在农科院地图上优越性的结论。

## 实际测试与构建

云端工作目录 `/workspace/work/agt_field_test_workbench`，独立实验 overlay
`/workspace/work/agri-ws`；容器 Ubuntu 22.04 / ROS 2 Humble。
原已有导航依赖树保留，仅在独立锁定副本应用组合补丁进行构建，补丁和哈希随主仓库交付。

```bash
PYTHONPATH=.:agt_field_test_bringup:agt_dynamic_map_cleaner:agt_robot_studio \
  QT_QPA_PLATFORM=offscreen python -m pytest -q tests agt_robot_studio/test
# 126 passed, 16 skipped, 0 failed；另 8 个 subtests passed。
# 跳过：11 个本实验原生/ROS 用例、4 个已有 ROS 用例（下方另行实际运行）；1 个真实 bag metadata 用例。

# Humble + 既有 interfaces/C1 overlay + 实验 overlay
source /opt/ros/humble/setup.bash
source /workspace/work/humble-ws/install/setup.bash
source /workspace/work/agri-ws/install/setup.bash
export AGT_FIELD_TEST_WS=/workspace/work/agri-ws
export LD_LIBRARY_PATH="$AGT_FIELD_TEST_WS/work/bbs3d_install/lib:$AGT_FIELD_TEST_WS/work/small_gicp_install/lib:${LD_LIBRARY_PATH:-}"
AGT_RUN_SC_ROS=1 AGT_RUN_SC_TESTS=1 python3 -m pytest -q tests/sc_kiss -s
# 11 passed, 0 failed, 0 skipped，全部实际原生库/ROS 接口。

AGT_RUN_ROS_INTEGRATION=1 ROS_DOMAIN_ID=188 QT_QPA_PLATFORM=offscreen \
  python3 -m pytest -q tests/ros --tb=short
# 4 passed, 0 failed：相机真实 Action/软件设备、超时、三视图消息/TF/Qt。
```

原定位依赖两包及系统启动所有权测试实际运行 **34 passed, 0 failed**：

```bash
python3 -m pytest -q \
 "$AGT_FIELD_TEST_WS/src/agt_navigation_v3/navigation/localization/agt_global_relocalization/test" \
 "$AGT_FIELD_TEST_WS/src/agt_navigation_v3/navigation/localization/agt_localization_manager/test" \
 "$AGT_FIELD_TEST_WS/src/agt_navigation_v3/bringup/agt_system_bringup/test"
```

实际通过的构建命令：

```bash
# small_gicp、CPU 3D-BBS 先按原固定依赖构建到 work/*_install
cd "$AGT_FIELD_TEST_WS"
colcon build --packages-select agt_sc_kiss agt_global_relocalization \
  agt_localization_manager agt_batch_lio_adapter agt_global_relocalization_native agt_robot_studio \
  --cmake-args -DSCANCONTEXT_SOURCE_DIR=/workspace/work/external/scancontext -DCMAKE_BUILD_TYPE=Release
# 6 packages finished；真实 KISS 扩展另由 scripts/build_kiss_optional.py 成功编译。
colcon build --base-paths src/agt_navigation_v3/bringup/agt_system_bringup \
  --packages-select agt_system_bringup
# 1 package finished（不代表缺少的车辆 runtime 依赖已经部署）。
QT_QPA_PLATFORM=offscreen colcon test --base-paths src/agt_field_test_workbench/agt_robot_studio \
  --packages-select agt_robot_studio --event-handlers console_direct+
colcon test-result --all --verbose
# 33 tests, 0 errors, 0 failures, 0 skipped；与主机 GUI 用例重复，不累加通过数。
```

额外实际执行 Python compileall（独立临时 pycache）、Bash `-n`、Git whitespace 检查、
组合补丁重复应用/未知修改保留测试和源接口导入检查。

未通过的环境构建也保留证据：对全部 `src` 选择系统/整车 bringup 时 colcon 因车辆运行依赖未构建而拒绝；
只选择目标源码后系统 bringup 成功，整车 bringup CMake 继续因缺少 `livox_ros_driver2Config.cmake` 失败，
尚缺 Livox/整套车辆与 Nav2 部署 overlay。没有伪造完整整车构建或 FAST-LIO/Nav2/CAN 集成结果。
PCL、SC、KISS、GICP、BBS 和定位 Manager 本身均已实际构建。
初次调用也曾遇到旧容器代理/缺少 GNU time，已在当前容器修复并重跑；未关闭 TLS 校验。

## 农科院上车顺序及回退

1. 在已稳定的工控机 baseline 上保留原地图/标定/BBS 资产和依赖补丁，构建本可选分支；先回放同一原始 bag，检查连续 FAST-LIO 跟踪、时钟、安装 TF、地图哈希与 SC DB 预处理一致。
2. 静止、禁运动地做同查询 A/B；记录成功、SC/KISS/GICP 时间、Fitness/Overlap/内点、CPU/RSS。无测量真值的误差统一 `NOT_RUN`。
3. 选至少 **5 个**已知测试区域启动点，每点先 BBS 再 SC+KISS，各做至少 3 次冷启动；无人为初始位姿。现场监督人员核对地图位置/航向，保存 bag、状态 JSON、地图/DB/标定身份和失败记录。
4. 每次确认定位正确且原许可/安全报告有效后，才通过原门禁进行低速短距离导航，观察 TF 跳变、定位丢失、遥控优先、超时停车；随后运行原路线任务和相机任务回归。
5. 专门测试无候选、错误 DB、传感器过期、机器人移动、超时与停止，确认无有效位姿、无新增运动许可、无配准孤儿进程。

SC+KISS 失败时：保持停车 → 停止定位/导航会话 → Qt 选 BBS（或 `global_backend:=bbs`）→
用原 PCD/标定/原资产重新请求定位。无自动多级回退，不覆盖或激活地图；不要删除稳定资产或合并 main。

**是否具备现场实验条件：软件链已具备接入和离线回放条件；完整工控机依赖部署、真实农科院 DB/地图校验、
真实 bag 和上述五点人工监督验收仍待完成，当前不能判定实车可直接运动验收。**
