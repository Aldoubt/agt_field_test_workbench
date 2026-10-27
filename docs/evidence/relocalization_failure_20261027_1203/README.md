# 2026-10-27 12:03 当前停车位置重定位失败

状态：本轮失败已由日志确认；物理位置坐标与相似原因待核实。

操作人员原话：“记录下刚刚这个当前这个位置的重定位失败和最佳一个很相似”。暂按“与上一个重定位失败现象很相似”归档，保留原话，不将相似性当作同一根因的结论。

## 本轮证据（北京时间）

- 12:03:52.120：QUERY_READY，静止查询就绪。
- 12:03:52.408：BBS_SEARCHING；原始筛选点 46089，体素后 6451；范围 0.5–30 m、体素 0.25 m，mapping_body；后端 polar_context_bbs_gicp。
- 12:03:56.090：BBS_COARSE_FOUND。
- 12:03:56.091：REJECTED，`fitness gate failed: 1.011`，总耗时约 3.989 秒。当前安装配置的 max_fitness=1.0，结果未达到质量门限。
- 后续 manager 为 waiting_global_pose，global_correction_valid=false；tracker 为 WAIT_LOCALIZED / manager_not_localized。
- 日志中的地图 SHA256：5ffba4e1b9bd95c6233d744e6b6d496c2e91f6e5d24ac2e2a4eea23e5c08ec99。

## 与之前失败的区别

此前 11:18 的失败为 BBS 没找到有效位姿，候选提示 003294.pcd；更早一轮是 GICP 未收敛。本轮已得到粗配准，最终在 fitness 质量门限被拒绝。三者现场表现都可能是无法定位，但失败阶段不同，不能直接认定同因，也不应仅因 1.011 接近 1.0 就放宽门限。

这是独立于 11:41 导航 Action 确认超时的重定位问题。此处仅记录“本轮在当前地点失败”；没有独立参考位姿，不能声明整个区域无法定位或记录一个未经验证的 map 坐标。

## 原始证据索引

工作区根目录：/home/yangxuan/agt_field_test_ws。

- runs/20261027T040336Z-4a01a844/process.log
- runs/localization_20261027T040341Z_c883ea95.jsonl
- 同目录 matcher_excerpt.log 保存本轮匹配日志摘录。
- 此次没有新启动 rosbag，也没有将较早的 bag 冒认为本轮匹配记录。

未修改定位门限、未发布定位或导航指令。
