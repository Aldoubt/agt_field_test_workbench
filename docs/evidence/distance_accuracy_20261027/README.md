# 2026-10-27 距离测试批次与终点诊断

## 数据与事实

分析 bag：`bags/field_trial_20261027T013018Z_dca99a3a`，约 89.19 秒。
任务记录：`data/captures/missions/accuracy_5m_20261027_093015`。
启动日志：`runs/20261027T012826Z-5a1735aa/process.log`。
分析程序及提取结果：`work/distance_review_20261027/`（只读解析 SQLite/CDR，未回放到机器人）。

启动模式是 distance。但本次任务文件内容与旧 `missions/distance/accuracy_5m.yaml` 一致：HOME_START=(18.329844,-9.197155)，而新批次 `missions/distance_batch3/start_pose.yaml` 记录起点=(19.329440,-9.122693)。任务只执行 HOME_START，90.91 秒后取消，completed_points=0，未执行 D5m。因此不能用此包声称完成了 5 m 精度测量。

以下误差按 NavigateToPose feedback 的 map 位姿与任务 HOME_START 计算，不是尺量真值；时间相对 bag 首条消息：

| 时间（秒） | 距任务起点误差 | 朝向误差 | 观察 |
| --- | --- | --- | --- |
| 约 5 | 1.007 m | 1.67° | 控制器已给负速度，说明倒车配置生效 |
| 约 15 | 0.862 m | 56.65° | 碰撞预测后进入旋转恢复 |
| 约 30 | 0.342 m | 124.76° | 绕回目标 |
| 约 50 | 0.0396 m | 14.66° | 指令约 0.03 m/s，轮速和局部里程计速度接近零 |
| 约 80 | 0.0570 m | 15.15° | 仍未满足到点条件，低速尝试继续 |

最小任务位置误差 0.03642 m，此时朝向误差 14.76°；最大恢复计数为 7。日志明确记录 collision ahead、controller patience exceeded、Running spin、Failed to make progress、Running wait。

bag 中只有 3 条 `/plan`，末点均为 (18.325002,-9.175000)，与任务目标相差 0.02268 m。该偏移来自栅格末点；RPP/goal checker 使用路径末端，因此这是任务目标与控制目标之间的额外误差来源，不能把路径末端的 2 cm 判定直接等同于相对任务坐标的 2 cm。

仅凭该包无法断定低速时的微小位移是底盘死区、打滑、外部仲裁还是定位处理造成，不能据此直接加大最低速度。bag 比任务晚约 3.86 秒开始，下一轮应先开始录包再发车。

## 修改

- accuracy 默认新建唯一批次目录；成功完成所有文件后原子更新 `work/active_distance_batch.json`。run/home 默认读取当前批次，缺少当前批次时拒绝隐式使用旧目录。显式 --directory 仍支持历史批次。
- 距离模式使用 Smac2D 派生规划器，仅在原生规划完成后补上精确目标。末段最长限制为原生容差加一个栅格对角线，按不超过 5 mm 间距检查中心代价和朝向 footprint，未知或占用则报错。
- 距离 NavigateToPose 行为树保留无效路径重规划和碰撞检查，去掉自动 Spin/BackUp/Wait/清图恢复。失败直接返回，交由任务运行器确认停稳。
- 距离进展检查改为 5 秒内至少 1 cm 平移或 0.05 rad 转动；不再使用 15 秒窗口。位置 2 cm、朝向约 5° 门限保持。
- 允许倒车且关闭 rotate-to-heading 的互斥配置保持。该配置没有主动原地终点朝向对齐，朝向不满足仍会失败，不能保证任意返程姿态收敛。

## 验证与后续

隔离 ROS_DOMAIN_ID=183、localhost 环境下规划回归：8 项通过，覆盖 5/8/12 m 精确末点、障碍/未知区域绕行、无路拒绝、精确目标处障碍或未知时拒绝。未向实车下发指令。

命令与配置回归：27 项通过，7 项因隔离条件跳过、3 项未选择。软件验证不等同于实车精度验收。

重新启动距离模式，停车对准测试线后执行 `./field_distance accuracy --distances 5 --trials 5`，先启动 `./field_trial record` 再执行 `./field_distance run 5`。停稳尺量后再 `./field_distance home`。保留新批次任务和独立纵横向误差，后续才能分解真正的 5 m 规划/跟踪/制动/定位误差。
