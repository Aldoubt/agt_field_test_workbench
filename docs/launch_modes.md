# 分开启动与三个组合启动入口

原来的 `field_trial`、`field_photo` 分终端启动方式全部保留。新增组合入口只把既有组件按顺序启动，不替换定位、Nav2 或任务协议。先 `cd ~/agt_field_test_ws`。

| 入口（选一个） | 启动内容 | 实际任务触发 |
|---|---|---|
| `./field_start distance` | 雷达、底盘、定位、Nav2、精度任务执行器、导航 RViz | `./field_distance run 5` 等显式命令 |
| `./field_start nav` | 雷达、底盘、定位、Nav2、导航 RViz | 在 RViz 手动发送导航目标 |
| `./field_start photo` | 导航链路、相机云台、拍照执行器、点位队列、拍照 RViz | **`./field_photo start`** |

三个组合入口都应在各自终端保持前台运行，互斥使用。已有旧链路时继续用分开命令，组合入口不会接管或停止它，也不会再开第二套底盘、TF 或定位。切换组合模式时先取消当前任务、观察停稳，在原组合终端 Ctrl+C 后启动另一个模式。

Bunker 现场导航的局部障碍层使用预处理后的点云标记、原始导航支路点云投影射线清理；人站着时保留障碍，人离开且有背景/地面射线观测时清除旧位置。传感器断流不会按时间让障碍消失。静态地图上的障碍仍属于地图，局部射线不会修改静态地图。全局/局部软代价膨胀半径为 0.80/0.65 m，衰减因子 3；车体 footprint 和 padding 保持原尺寸，RPP 的代价距离/衰减随局部膨胀同步。配置在核心链路下次启动时加载，不能只重开 RViz。

## 启动顺序

组合入口默认复用现有 `--hold-system-clock` 助手，主机已同步后暂停本轮 NTP 服务，退出恢复。若需要 sudo 密码，在启动终端输入。密码认证、时钟保持与退出恢复由前台父进程负责，后台组件不询问密码。相机在本地里程计就绪、时钟保持已进入会话之后才启动。

准备期间保持机器人静止。入口激活静态地图，检查新鲜本地 odom 和实测停车，只请求一次正常全局重定位；等待有效定位与 TF 后激活 Nav2，再加载该模式的执行器/RViz。重定位失败时不循环重试、不伪造时间戳、不发送车辆目标。默认 BBS，可用 `--global-backend sc_kiss` 显式选择既有 SC/KISS；地图用当前 `config/field_trial.yaml`，也可 `--map-id` 覆盖。

希望保留手动重定位步骤时：

```bash
./field_start photo --manual-localization
# 另开终端，静止后执行：
./field_trial relocalize
```

`--no-rviz` 只启动后台组件，之后仍可用旧 RViz 命令；`--no-clock-hold` 跳过时钟助手；两个计算预算可通过 `--startup-timeout-sec`、`--relocalization-timeout-sec` 设置，不修改里程计安全门槛。`--dry-run` 只输出启动计划，不创建节点、子进程或锁。

每个组件输出到启动终端打印的 `runs/<id>/process.log`。组合计划和阶段记录在 `runs/field_mode_*/session.json`。关闭 RViz 窗口不关闭导航链路，可重新单独打开。退出组合入口只关闭本次创建的进程组，不扫描/杀死其他会话。

必须等启动终端最终显示 `导航已激活。使用 ./field_distance ...`（或对应模式提示）再提交任务。仅看到 RViz、任务服务可用或 `map_tracking_ok`，仍可能处于准备阶段。组合入口在 `work/field_mode_state.json` 原子记录进程身份及就绪状态；`field_distance run/home` 和 `field_photo start` 会拒绝尚未就绪或正在退出的组合会话。过期进程记录不会阻碍原来的分开启动方式。

只读参数查询使用同一个客户端，服务发现最多等待 8 秒；单次响应等待 2 秒，最多尝试 3 次。首次响应丢失后会打印重试信息，迟到响应不会被当作下一次查询的结果。生命周期激活、重定位和参数写入不会自动重发。全部尝试失败仍会报告启动失败并清理本次组件，不显示就绪。

## 独立距离测试

距离模式不启动相机，使用原通用到达判定（当前 0.08 m / 5°），不会套用拍照模式 0.35 m 的位置容差。它复用原 MissionRuntime 的导航、停车确认和记录过程。

`field_start distance` 自动选择 `planning_profile=distance`：继续使用 SmacPlanner2D，去掉平滑过程中对原栅格折线的吸附，并增加有界平滑迭代次数。这样空闲直线上不再保留 8 邻域栅格搜索的左右折弯；全局膨胀同时保护未知区域边界，搜索仍禁止进入未知区域。障碍物代价、机器人尺寸、局部碰撞检查、速度和到达容差继续使用现有配置。普通 `field_start nav/photo` 和默认分开启动继续使用原规划配置。

这是距离测试专用配置，不强行穿越障碍。场地有障碍时仍可能绕行；若未知区挡住通道，应选择已观测的空地。原分开方式如需使用此配置，在核心启动命令追加 `--planning-profile distance`。配置只在下一次启动核心链路时加载，不会在线替换正在执行的路径。

启动终端：

```bash
./field_start distance
```

等待上述最终就绪提示，并确认现场测试方向后，操作终端：

```bash
./field_distance teach home
./field_distance accuracy --distances 5 7 10 --trials 5
```

teach 读取当前停车位姿；accuracy 生成 `missions/distance/accuracy_5m.yaml`、7m/10m、独立 `return_home.yaml` 和空白实测 CSV。这两条不发送目标。5/8/12 米同样支持：把 `--distances` 换成 `5 8 12`。

显式执行一轮、独立尺量、再返程：

```bash
./field_distance run 5
# 任务结束并停稳后进行独立尺量，填写 CSV；人员离开后再返程：
./field_distance home
```

每轮复用同一 home 和同一组目标，避免累计起点漂移。分别把 run 的距离换成 7、10，各测 5 次；`--trials` 只生成对应数量的空白测量记录，不会自动循环发车或填入成功结果。Nav2 可能绕障，米数表示固定起终点间距离。

默认点位本 `config/field_test_points.yaml` 和输出目录都不会覆盖已有内容。新一批测试用新名字：

```bash
./field_distance --points config/distance_batch2.yaml teach home
./field_distance --points config/distance_batch2.yaml accuracy --distances 5 8 12 --output missions/distance_batch2
./field_distance run 8 --directory missions/distance_batch2
./field_distance home --directory missions/distance_batch2
```

已有同一地图、同一起点的距离任务时，重新启动后直接 `./field_distance run 5`，不必再次 teach 或生成文件。若任务刚下发组合入口就报参数响应超时并退出，目标会随清理取消；先解决启动失败并等最终就绪，再重试任务。本次故障证据见 [启动响应超时回归](evidence/startup_response_fix_20261025/README.md)。

目前 run/home 按 `config/field_trial.yaml` 中地图校验任务；使用组合入口 `--map-id` 覆盖时，也应让该配置对应同一地图后再生成距离任务。`run/home` 拒绝带相机视图的任务、失效定位和正在使用拍照行为树的执行器。取消距离任务用 `./field_distance cancel`，它请求父任务取消；等待最终结果和停稳，不能把 Trigger 的提交响应当成停车完成。

## 导航模式

```bash
./field_start nav
```

该模式没有相机、巡检执行器或拍照队列。显示导航就绪后，在 RViz 用普通导航工具发目标。原来的 `./field_trial status --seconds 5`、`record`、`cancel` 继续可用。

## 导航加云台相机模式

```bash
./field_start photo
```

显示就绪后，在 RViz 的 2D Goal Pose 工具依次排队。选点只记录目标，启动命令也不会自动跑队列。外部触发入口保持原样：

```bash
./field_photo start
./field_photo cancel
./field_photo clear
```

服务 `/agt/rviz_patrol/start` 与任务 Action `/agt/mission/execute` 均保持不变。流程为导航 → 实测停稳 → 左中右三张照片 → 云台反馈确认回中 → 下一点 → RETURN_HOME。拍照模式启动时会核实已加载回中和拍照到达配置。

## 原分终端方式

以下命令仍可分别在独立终端运行：

```bash
./field_trial start --global-backend bbs --hold-system-clock
./field_trial map
./field_trial relocalize
./field_trial status --seconds 5
./field_trial nav
./field_photo camera
./field_photo runtime
./field_photo queue
./field_photo rviz
# RViz 排队后：
./field_photo start
```

精度测试的独立执行器仍可按 [旧距离测试文档](accuracy_and_photo_tasks.md) 启动；普通导航不需要相机相关四条命令。继续分开启动时，不需要运行 field_start。

新检出工作区运行 `scripts/init_workspace.sh` 会补齐四个根目录包装器；已有包装器和现场配置保留，依赖版本不变。原导航阶段标签是 `field-long-distance-navigation-pass-20261025`，组合入口开发放在它之后，标签保持指向原快照。
