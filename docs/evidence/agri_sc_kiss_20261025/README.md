# 实际执行证据

基线 `7039bf7fdfba627b84e76be255b1a2c5e9a6ef48`，2026-10-25 云端 Ubuntu 22.04/Humble 容器。
所有算法结果来自锁定原生库，不使用替身配准器。输入为独立合成场景，不是农科院地图。

- `host_tests.txt`：126 passed / 16 skipped / 0 failed，另 8 subtests；其中需 overlay 的 15 项另行运行，真实 bag 用例仍跳过。
- `native_build_and_ros.txt`：6 个定位/实验/Qt 包构建成功；真实 SC/KISS/GICP、ROS Manager、失败拒绝、运动取消、进程超时回收、两模式 Launch 参数，共 11 passed / 0 failed。
- `baseline_and_ownership.txt`：系统 bringup 单包成功构建，既有定位/启动所有权 34 passed；包含当时 Qt 验证。
- `qt_colcon.txt`：最终 UI 重建后 colcon 实际运行 33 项，0 errors / failures / skipped。
- `existing_camera_views_ros.txt`：原相机 Action 与三视图真实消息/软件设备回归 4 passed / 0 failed。
- `kiss_locked_build.txt`、`kiss_build_metadata.json`：现有锁定配方实际构建 KISS Python 扩展；成功产物随后参与所有配准。
- `scancontext_native_build.txt`、`small_gicp_build.txt`：真实原生编译；SC 五个源文件哈希在仓库锁中。
- `comparison.json`、`spec.json`、`contract.json`、`ab_results.md`：最终同查询 A/B。绝对输入路径是本次云端产物位置；用文档 `--synthetic` 命令可重新生成，不需要复制二进制数据。
- `original_bbs_identity_success.json`：原整图 BBS 对单一水平合成场景确实成功，耗时约 27 秒；原候选 BBS 的 ROS 接受结果在主测试日志。
- `full_overlay_missing_dependencies.txt`：整车包在完整 src 图中因车辆 runtime 依赖未构建而受阻。
- `field_bringup_environment_block.txt`：缩小源码集合后进一步验证到 CMake；缺少 Livox driver Config，1 package failed。完整车辆构建未通过。

初始倾斜合成场景：整图 BBS 30 秒超时；原候选 BBS 被原 Fitness 门控拒绝（1.229>1.00）。
最终 BBS ROS 成功回归使用水平查询，SC 使用非零 roll/pitch/yaw 查询；两者不是该项测试中的同查询性能对照。
同查询 A/B 单独保存在 comparison.json，不将两份不同夹具的结果相比较。

REAL_BAG_VERIFIED=NOT_RUN；HARDWARE_VERIFIED=NOT_RUN。
软件接受链验证不等同于真实 FAST-LIO 连续跟踪、Nav2 路线或工控机车辆验收。
