# PR #10 与 2026-10-29 现场基线整合

整合基线为 Workbench `e5b3620ccc5cc00602a9033f705ab86a483e7e79`，原 PR 为
`3ee540272f8578640ad2a2faff91a1a0197616d5`。保留两侧提交历史。

## 本次行为

- 保留 Qt 三主题、目标草稿、航点编辑、相机 job/tag 关联、链路诊断及完整动作身份协议。
- 保留 main 的 1.20 × 0.95 m Bunker footprint、15°安装角、平面 RPP、精确距离规划、
  后杆过滤盒、全局未知区保护和长序列拍照行为树。
- Qt 的 inspection runtime 和 CLI 共用 `photo_runtime_parameters`：拍照行为树、回中、
  15 秒停车等待一致；Bunker 显式启用控制等待和轮速停车门。FW-mini 不等待 Bunker 反馈，
  保留其已有停车判断，尚未宣称完成 FW-mini 拍照实机验收。
- 拍照 BT 安装到 bringup 的 share/config；Workbench 会话明确传入本仓库 BT。
- RPP 的膨胀因子从实际 local costmap 取值，当前均为 2；PhotoFollowPath 同步继承。
  膨胀半径仍为 local 1.00 m / global 1.20 m，cost_scaling_dist 仍为 0.85 m。
  该修正会影响近障减速估算，需现场复测，不能将旧参数现场结果视为新参数验收。
- Qt“相对移动”沿用普通导航；5/7/10 m 精度测试沿用启用 distance 配置的 CLI。
  默认从当前位姿生成目标，指定 `--directory` 才复用固定批次。

## BBS 和依赖边界

BBS 候选恢复实现、manifest、测试和历史证据保留为独立实验；默认初始化和构建不再自动应用。
`apply_bbs_recovery.py` 仍要求其审查过的 `agt_navigation_v3@b3a6048`，并保留文件哈希检查。
显式安装该补丁的独立实验工作区使用 `audit.py --with-bbs-recovery`。

现场导航版本为 `3dd05c173aaea2b82b3a2231e6eaa38792b4a2da`。不要为了满足旧补丁检查而
回退现场依赖，也不要跳过哈希检查。完整依赖锁迁移仍是后续工作：见
[现场版本记录](evidence/field_long_sequence_20261029/source_versions.json)。
本次工作台合并不代表旧 `.repos` 已能重建全部现场成果；初始化/全量构建脚本仍会拒绝
与旧锁不符的已有依赖，保护现场源码。

## 本地验证

组合回归在无 ROS 的 Python 3.12 / PyQt5 环境运行：269 passed、1 skipped、3 deselected、
79 subtests passed。包含真实 Qt 离屏窗口、HTTP、动作状态机、相机/路线交互及组合配置测试。

- skip：未提供真实 B+C bag metadata。
- 两个 Unix RPC 测试：此执行环境创建 AF_UNIX socket 返回 EPERM。
- 一个孤儿进程组测试：此执行环境无法取得子进程的 `/proc` start ticks，实际子进程存活
  检查不具备标准 Linux 行为。保持测试和生产逻辑不变，交由 GitHub Ubuntu CI 完整运行。
- Python 编译、Shell 语法和 `git diff --check` 通过。
- 新增 9 项组合测试实际运行配置生成器，使用现场导航 `3dd05c1` 的配置，并验证 launch
  给 mission_runtime 传入的参数表达式；Nav2 BT 模板为测试夹具，不等于真实 ROS 构建。
- GitHub CI 增加完整动作协议与组合配置检查，包含本地受限的 Unix RPC 测试。
  最终远端结果以本 PR 当前 HEAD 的 Checks 为准。

本次没有连接机器人，没有运行 ROS Humble 原生编译、真实 DDS、BBS 原生构建或实车任务。

## 现场更新和最小复测

1. 正常结束任务和旧会话，记录现场 SHA、地图、标定、参数及未提交改动。
2. 更新 main，只构建 `agt_robot_studio` 和 `agt_field_test_bringup`；保留已经验证的导航依赖。
   不使用旧锁初始化脚本重置现场工作区。具体命令见 [试用说明](operator_ui_trial.md)。
3. 同步重启 Workbench、ROS Bridge 和 Qt。只重开 Qt 不会替换常驻后端或旧导航参数。
4. 核对实际 footprint、15°外参、平面 RPP、局部/全局膨胀因子、PhotoFollowPath、拍照 BT、
   wheel_stop_gate 和 wait_for_bunker_control。旧配置快照不会因模板更新自动改变。
5. 空旷处验证草稿不运动、显式提交、执行中停止、迟到回执、单点和多点拍照返航。
6. 在已验证水泥地重新记录 5/7/10 m；加入一个新增障碍场景检查净距、减速与重新规划。

Qt 仍使用正式会话的地图、标定与安全报告检查。现场 CANDIDATE 标定不能通过修改状态绕过
正式检查；需要继续使用现有受监督 CLI 入口，或按既有流程完成标定验收。
