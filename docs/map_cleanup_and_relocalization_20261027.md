# 点云清理后生成并加载新的导航地图

本次修复基于 Workbench `1c35e1b4f9e96f84cd8ee62c7672785d5b152b81`，保留当天已提交的 Bunker 尺寸和后杆过滤修改。建图依赖 SHA、FAST-LIO 输入、逐点时间、PGO 开关和定位质量阈值保持原样。V3 的新增内容通过精确 SHA 补丁安装。

## 1. 先区分这次拒绝发生在哪里

仓库中的 [12:03 重定位证据](evidence/relocalization_failure_20261027_1203/README.md) 记录了 `BBS_COARSE_FOUND`，随后 `REJECTED: fitness gate failed: 1.011`，当时 `max_fitness=1.0`。这一次是精配准结果未过质量门限；不能与 native `GICP did not converge` 当成同一种失败。这个实现的 fitness 是 small_gicp `error / num_inliers`，不能直接当作米或通用 PCL ICP 均方误差。

地图自波是待验证的原因。这里补齐清理与加载链路，不放宽 `max_fitness`，也不宣称已证明现场原因或提升成功率。

## 2. 本次修复解决的关联问题

| 环节 | 修复后的行为 |
|---|---|
| 地图与点云选择 | Qt 按源 PCD 路径与 SHA 关联；找不到时清空选择，避免清理另一张地图。 |
| 清理后预览 | 自动选中新 cloud；旧请求迟到时不会覆盖新预览；等待新版本期间不能继续清理或投影旧版本。 |
| 三维删点 | 仍由 PCL 处理完整 PCD，新目录保存新版本，父版本不覆盖。 |
| 关键帧 | 按每帧 `T_map_body` 在地图坐标判断同一 crop/delete ROI，保留剩余原始 body 坐标点记录、字段、位姿和时间；生成新 patches。 |
| 描述子 | Polar Context 和 Scan Context 都使用新关键帧；缓存同时校验位姿与 patch 内容身份。清理失败不会偷偷回退原 patches。 |
| BBS/GICP target | 从新 cloud 构建新 `global_map_downsampled.pcd` 与 `voxelmaps_coords`，新版本拥有独立资产目录。 |
| 查询自波 | 可选 JSON 指定 `base_link` 车体盒；全局重定位 query 在体素化前过滤，原始 Livox→LIO 数据不变。 |
| 启动和复用 | 地图资产与 query 的过滤配置必须匹配；会话冻结 JSON、保存 SHA；配置变化需要明确重启定位。 |

## 3. 已部署工作区的更新方法

先在工作区仓库检查当前分支和修改。下面示例使用本修复分支；存在现场未提交修改时先保留它们，不使用 reset 或强制覆盖。

```bash
cd ~/agt_field_test_ws/src/agt_field_test_workbench
git status --short
git fetch origin
git switch --track origin/fix/map-cleanup-query-consistency-20261027
```

已有完整 Humble 部署时，可以在停止旧定位及工作台进程后增量应用补丁和构建：

```bash
source /opt/ros/humble/setup.bash
source ~/agt_field_test_ws/install/setup.bash
cd ~/agt_field_test_ws/src/agt_field_test_workbench
python3 scripts/apply_control_integration.py --workspace ~/agt_field_test_ws
python3 scripts/apply_sc_kiss_integration.py --workspace ~/agt_field_test_ws
python3 scripts/apply_control_integration.py --workspace ~/agt_field_test_ws \
  --manifest patches/relocalization_self_filter_manifest.json
cd ~/agt_field_test_ws
colcon build --base-paths src --packages-select \
  agt_field_test_bringup agt_global_relocalization agt_system_bringup agt_robot_studio
source install/setup.bash
```

使用 Scan Context + KISS 时，还需更新它的 Python 预处理和数据库构建代码：

```bash
cd ~/agt_field_test_ws/src/agt_field_test_workbench
bash scripts/build_sc_experiment.sh
```

完整部署入口 `scripts/build_humble.sh` 和 SC 入口均包含新补丁。补丁安装器只接受锁定版本及已审阅的后继 SHA；遇到未知源码修改时停止并保留文件。更新后重新启动 Workbench 和 Studio，避免新界面连接旧预览 API。

## 4. 推荐先做一张仅删点的候选地图

1. 在 **地图** 页选择旧地图并点击 **加载版本**。上方 PCD 应关联到同一源文件，核对名称、cloud ID 和 SHA。
2. 在 **三维 PCD 全量清洗** 中确认残影位置。XYZ ROI 使用 **map 坐标**，单位米；它与机器人 `base_link` 下的车体盒是两套坐标。
3. 先选小范围删除，点击 **删除框内杆件/残影 → 新 PCD 版本**。每次都核对新预览和点数，保留墙、柱、树干等稳定结构。
4. 查看 **关键帧清理状态**。`APPLIED` 表示此次 ROI 已同步到描述子源；`UNAVAILABLE`、`UNSUPPORTED` 或 `FAILED` 表示目前只有融合 PCD 候选，不能当成完整的描述子清理成功。
5. 选中新 PCD，点击 **生成二维地图**。投影沿用当前锁定的 `local_ground` 后端，同时为该 cloud 构建定位资产。检查 BBS、Polar、SC 各自状态；使用哪个重定位后端，就要求其对应资产成功。
6. 在 **定位与导航** 选择新版本，停止旧定位并启动新定位观察。检查实际加载的 map ID、PCD SHA 和资产目录，确认没有沿用旧进程中的 target。DRAFT 可用于定位观察。
7. 在同一段数据或同一静止位置比较旧/新地图的结果。验证完成后再按现有流程保存/选用导航版本；新二维地图的路线需重新保存到对应版本。

仅擦 PGM 像素不会清理 GICP 的三维 target。也不要只覆盖某个同名 `map.pcd`：当前 native 后端会优先使用资产目录中的降采样地图。

## 5. 再做实时 query 自波过滤对照

默认 `query_self_filter_config` 为空，保持原行为。已有 `navigation_self_mask_boxes` 原先用于障碍预处理；本次不会未经选择就把它当成定位过滤参数。

下面命令**只生成新的候选 JSON**，不启用配置，也不修改 robot profile。它读取 `config/field_trial.yaml` 所选的 profile、标定 bundle，并显式复制当前导航盒：

```bash
cd ~/agt_field_test_ws
python3 src/agt_field_test_workbench/scripts/make_query_self_filter.py \
  --workspace ~/agt_field_test_ws \
  --config config/field_trial.yaml \
  --use-navigation-boxes \
  --output work/query_self_filter_candidate.json
```

如果有另行核对的定位专用车体盒，用 `--boxes-file work/body_boxes.json` 替换 `--use-navigation-boxes`。文件内容为平铺列表或 `{"boxes": [...]}`，每六项依次为 `xmin,xmax,ymin,ymax,zmin,zmax`，均在 `base_link` 中。输出存在时拒绝覆盖，修改参数需另存新文件。

脚本针对所选 bundle 中的 FAST-LIO2 内参生成 `T_base_body = T_base_lidar × inverse(T_body_lidar)`。`base_link_height_m` 属于另一个 TF 边，不叠加到这个变换。现场当前杆件盒范围及遮挡情况仍需核对；`CANDIDATE_NOT_FIELD_VERIFIED` 不代表已测量通过。

在 Qt 参数页填候选 JSON 路径，然后从**未做此 mask 的源 cloud** 创建新候选：可以在导入时附带配置，或对源 cloud 执行实际需要的 ROI 清理。后端会给新 cloud 冻结一份 JSON，同时对关键帧应用 mask。随后重新生成二维地图及描述子。加载这个新版本时，界面会带出其冻结配置；无需重新填路径。

这里有两个范围区别：

- 车体 mask 会清理 body-frame 关键帧和在线全局重定位 query。它不会凭一个固定盒自动从已融合的全局 PCD 删除沿轨迹移动的车体残影；融合 PCD 的污染仍需 map ROI 清理，或回到逐帧数据重新导出。
- 本次不改 Map Tracker 的在线输入。不能把全局重定位过滤结果当成连续跟踪链也已统一过滤。

配置的 `T_base_body` 与运行外参不符、查询 body 转换与活动 LIO 不符，或缺少输入点云时间戳处的 TF 时，该次启用的查询会失败并报告原因，不退回未过滤数据。SC/BBS 启动也会拒绝新 query 与旧描述子、或旧 query 与新描述子混用。

已经用某个非空 mask 删除过关键帧的 cloud，不能靠清空或改变配置恢复那些点。需要另一组 mask 时，回到保留的未过滤源版本创建分支。

命令行 `field_trial check/start` 和 `field_start` 支持 `--query-self-filter-config /absolute/candidate.json`，未传参数时按配置/所选地图记录选择。`check` 沿用现有 ACCEPTED 地图要求；DRAFT 候选定位观察使用 Qt。显式传空字符串会关闭请求侧 mask，但如果资产使用了 mask，契约校验会拒绝启动。

## 6. 失败状态与验证范围

关键帧读取器支持标准 ASCII 和 binary PCD；`binary_compressed` 明确标为 `UNSUPPORTED`，不会重新使用未清理的旧 patches。融合 PCD 的 PCL 处理能力保持原样。空关键帧会被记录并从位姿列表移除；SC 进一步跳过预处理后少于 100 点的帧，全无可用帧则构建失败。

同步的是本次 ROI 和可选 mask。此前 temporal cleanup 的历史逐帧判定不会被猜测或自动重放。体素采样不重新写 body-frame patch 的点字段；同一操作批次要求 ROI 在 voxel 之前。为避免已有体素质心与原始点边界不同，推荐从保留的全分辨率 cloud 完成几何删点后再降采样。

回归覆盖了保留父资产、连续清理、字段与时间不变、缺帧/不支持编码、输入变动、缓存失效、旧 DB 拒载、过滤配置冻结与双向匹配、补丁幂等性，以及界面版本/预览选择。原生 PCL/描述子命令在编排单测中使用替身；实际 V3 `merged_points()` 在独立准备的固定版本源码上另行执行。Qt 测试需要 PyQt5；最终通过情况查看本 PR 的 CI。

真实 bag、新地图的实际 native 构建、ROS launch、GICP 质量和现场定位结果尚未验证。对照时固定 query、外参、范围/体素参数和门限，记录候选阶段、`converged`、inliers、fitness、耗时及位姿；不要仅以 fitness 略低就判断位姿正确。
