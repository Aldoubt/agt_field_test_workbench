# BBS 失败证据核查与候选恢复改动

> 2026-10-29 整合说明：本补丁保留为旧锁定依赖上的独立实验，默认初始化和构建不自动应用。现场导航 `3dd05c1` 尚未完成补丁迁移；不要降级现场依赖。审计显式实验工作区时使用 `scripts/audit.py --with-bbs-recovery`。见 [整合说明](../../pr10_integration.md)。

本记录审查的是仓库已有的 2026-10-25 现场日志，以及锁定到
`agt_navigation_v3@b3a60487dff973c6326aca2ca2e9db1e78daeaaa` 的实现。
2026-10-26 的分支日期不代表重新进行了现场实验。没有本次失败的精确查询点云，
因此不能宣称已精确复现原失败、确定其唯一根因，或已经完成实车验证。

## 已有现场证据

原始归档见 [localization_open_area_20261025](../localization_open_area_20261025/README.md)。

| 项目 | 20:52 失败 | 20:53:54 近树位置成功 |
|---|---|---|
| 会话 | `20261025T125138Z-81e55b1b` | `20261025T125344Z-d54b43e1` |
| 后端 | Polar Context → BBS → small_gicp | 同左 |
| 结果 | GICP 未收敛，return code 4 | `SUCCEEDED` |
| 总耗时 | 14.262 s | 2.064 s |
| 查询点数 | 16,220 | 11,989 |
| 范围过滤后、体素化前点数 | 48,490 | 60,129 |
| 查询契约 | `mapping_body`；0.25 m；0.5–30 m | 同左 |
| 选中 patch | 失败日志未保留 | `003334.pcd` |
| GICP fitness / overlap | 失败日志未保留 | 0.199488 / 0.987252 |

失败时本地里程计 10 Hz，时间戳年龄 p50/p95/max 为 37.02/46.54/73.61 ms；
`local_odom_fresh=true`。阻塞发生在全局匹配阶段，不能把它归因于里程计新鲜度。
失败点数多于成功点数，总点数本身不支持“输入点数太少就是根因”。

成功日志中的 `0.854535` 字段名是 **`score`**。该后端的分数计算为
`0.50 × bbs_score + 0.25 × overlap + 0.25 × max(0, descriptor_similarity)`。
原归档 README 将其称作“BBS score”不够准确；不能把复合分数当作独立 BBS 粗配分数。

失败后临时 `query_scan.pcd` 已被删除。相邻采集的五帧经相同固定外参重建出的查询
**不是后端当时的精确输入**，不能用它声称原失败已被复现。
另外，20:40/20:42 的“无有效粗位姿”、20:45 的成功、20:57 底盘
`free(): invalid pointer` 崩溃分别属于不同记录。

本次还检查了云端已有文件：`/workspace/agt-cloud` 的三份日志修改时间为 10 月 9 日，
内容是开发环境安装、契约测试和 C++ 检查；`agt-lio-pgo-mapping` 中有离线 TopK 分析工具，
没有找到此次故障的新运行日志、精确 query 或实车 bag。以上检查没有触发机器人动作。

## 源码中确认的可修复问题

基线 `candidate_bbs_gicp_localizer.cpp` 配置 `candidate_top_k=2`，但只对候选运行粗配，
然后仅对最高 BBS 分数的候选运行一次 GICP。BBS 分数至少 0.65 且描述子相似度至少 0.85
时，还会在 GICP 验证之前提前退出候选循环。该候选精配失败后不会继续试其余候选。

这解释了一个确定的流程缺口：候选列表中可能还有可用候选，但第一次细配失败会直接结束。
它与现场 return code 4 的路径一致；缺少精确输入和逐候选轨迹，仍不能证明第二候选一定
能救回原查询，也不能排除粗位姿、地图资产、场景结构或姿态残差问题。

分支修复范围是：

- 在现有候选预算内逐个运行 BBS 和 GICP；粗分足够高不再独自触发提前成功。
- 候选必须通过与 ROS 包装器一致的 score、fitness、overlap 和有限数检查，失败后尝试后续候选。
- 输出候选阶段及原因，保存精确 query、命令、帧契约、地图路径、进程输出、异常和 query SHA256。
- 默认仅保留最近 20 个已完成的本 schema 记录，跳过运行中记录、外来目录、损坏元数据和软链接。

这里的选择策略是“首个通过既有门限的完整候选”，不是对所有候选评优，也没有新引入 BBS
歧义检测或放宽质量门限。`mapping_body` 与 `base_link` 的显式契约保持：前者后端输出
`T_map_body`，包装器只转换一次为 `T_map_base`；后者显式转换候选且避免二次转换。

## 对参数建议的逐条核验

| 建议 | 当前代码事实 | 本次判断 |
|---|---|---|
| `score_threshold_percentage .9 → .7` | 候选 BBS 使用 `.05`；旧整图后端默认 `.30` | 不适用于当前基线；不照抄修改 |
| `query_voxel_leaf .25 → .15` | ROS 查询 `.25 m`；候选后端还会以 `.35 m` 下采样 | 需同一精确查询回放比较；不能据总点数判断修复 |
| Polar Top3 逐候选 BBS+GICP | 当前 BBS Top2 只粗配、单次精配；SC 已 Top3 逐候选 KISS+GICP | 先修完整候选尝试；K 和每候选时间需在总预算内实测 |
| `min_rpy ±.02 → ±.05` | 当前候选残余 roll/pitch/yaw 范围均为 `0°`，由描述子和候选姿态提供种子 | 所述基线不符；先核对单位、姿态残差与预算，再做参数扫描 |
| Top2 fitness 差 `<.02` 且 pose 差 `>2 m` 判歧义 | SC `localizer.py` 已实现 fitness 差及 XY 距离比较；BBS 没有 | SC 不能作为新增功能；BBS 如加入需评估假阳性/假阴性，不能把固定阈值当已验证 |
| GICP overlap 至少 `.2` | ROS 包装器及 SC 已有；SC 还检查有限数、fitness、Hessian 退化 | 保留现有门限 |
| 连续三帧在 `5 cm / 2°` 内 | 未实现该规则；已有静止检测，SC 发布前再检查静止状态 | 相关窗口的稳定错误也可能通过；硬数值需要数据，不能当正确性的证明 |
| SC 失败自动 fallback BBS | 界面明确 BBS 为显式备用，不静默切换 | 属于产品策略变更，不能伪装成 bug 修复；此次保持显式选择 |

对应实现入口：v3 的 `global_relocalization.yaml`、`candidate_bbs_gicp_localizer.cpp`，
本仓库的 `agt_sc_kiss/agt_sc_kiss/localizer.py` 与 `runtime.py`。

## 验证边界与现场补证

证据辅助模块已进行 11 项无 ROS 离线测试，覆盖 return code 4、超时部分输出、原异常保留、
query 哈希、保留规则、软链接/坏元数据、清理异常，以及 pose 已发布后归档失败不能反报拒绝。
这些结果验证归档逻辑，不验证配准精度、当前地图或实车成功率。
本分支还完成以下独立验证：

- Ubuntu 22.04 容器中，以锁定的 `3d_bbs@41529a3`、`small_gicp@2c6c783` 和 PCL 1.12
  编译、链接真实候选后端。此次编译使用独立 C++ 命令；未运行整套 ROS/colcon 构建。
- `tests/bbs_native/` 共 20 项通过：15 项编译并执行生产 C++ 的候选流程测试，仅注入
  BBS/GICP 返回值；5 项执行真实子进程，验证超时、取消、完整终止及部分输出保留。
  对照旧源码，同一“第一候选粗分高但精配失败”输入直接返回 4；新代码继续第二候选通过。
- 真实 BBS 和 GICP 在非对称合成地图完成回归烟测：平移误差约 1.3 mm，yaw 误差约
  0.0037°，overlap 1.0。数据与算法均真实执行，但这是受控合成输入，不能代表果园准确率。
  输出见 [synthetic_geometry_result.json](synthetic_geometry_result.json)，生成器为
  `tests/bbs_geometry_smoke.py`。
- `tests/test_bbs_patch.py` 的 2 项检查通过：锁定源码上按 control→SC→BBS 顺序应用，重复
  应用及前序补丁复检均成功；遇到未审查的本地修改则停止并保留修改。依赖 HEAD 保持原值。

共 33 项回归测试及 1 项真实几何烟测通过。ROS 集成、原失败精确回放和实车操作尚未验证。

## 部署与明日试验

先在正常工作区更新本分支，再执行仓库现有 `scripts/build_humble.sh`。初始化和构建脚本均已
接入 `apply_bbs_recovery.py`，它要求先应用现有 control 和 SC 补丁；无需手工改依赖源码。
包装器和原生候选后端必须一起重新构建，新包装器会传递当前质量门限到候选后端。
若依赖有未审查的本地改动，安装器会保留它们并停止，不能以覆盖改动解决。

验证已安装的真实几何烟测可在加载 ROS 与工作区环境后运行：

```bash
native_bin="$(ros2 pkg prefix agt_global_relocalization_native)/lib/agt_global_relocalization_native"
python3 tests/bbs_geometry_smoke.py \
  --assets-builder "$native_bin/build_relocalization_assets" \
  --candidate-builder "$native_bin/build_relocalization_candidates" \
  --localizer "$native_bin/candidate_bbs_gicp_localizer"
```

该命令只使用临时合成 PCD 和资产，不运行 ROS 节点或控制机器人。现场再用同一地图、同一停稳
位置分别做近树与空旷处的显式定位请求，检查接受/拒绝、位置是否正确及失败后的下一步提示。
不要用单次成功直接开始自动任务。

现场重新触发后应保存 `work_dir/attempts/attempt_*/`，同时记录地图版本/资产、所选后端、
当前标定、停稳时间和现场位置。先核对精确 query 与逐候选失败原因，再有针对性地比较
候选数、残余角度范围、体素大小。不同参数必须复用相同输入和地图；单次成功不能覆盖另一
位置的失败，也不能替代导航门禁及误接受检查。

状态消息的 `evidence_dir` 指向当前请求目录；默认路径为
`~/.ros/agt_global_relocalization/attempts/`。目录内包含 `query_scan.pcd`、`attempt.json`、
`backend.stdout`、`backend.stderr`。失败查询不会再随临时目录立即删除；需要长期保存的现场
证据应复制到该次运行报告目录，避免超过默认 20 次已完成记录后被正常清理。
