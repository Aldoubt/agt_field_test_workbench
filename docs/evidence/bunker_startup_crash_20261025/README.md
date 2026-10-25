# nav 启动时底盘节点崩溃修复

2026-10-25 20:57:10（北京时间），`field_start nav` 会话 `20261025T125708Z-e1d1367b` 在等待新鲜本地里程计期间退出。根日志记录 `agt_bunker_base_node` 的 `free(): invalid pointer`、SIGABRT / exit -6；随后 required-process 退出策略关闭其他节点。launch 自身返回 0，原启动提示只显示末尾正常退出行，掩盖子进程错误。本轮尚未进入全局匹配，不能当作空旷位置匹配失败。

core dump 主线程栈落在 `BunkerBaseRos::Run()`，反汇编对应 `BunkerRobot::Connect(port_name_)` 返回后销毁临时字符串的位置，同时存在两条 CAN io 线程。排查发现 `BunkerBaseRos::Initialize()` 在栈上创建 `ProtocolDetector`；其回调绑定裸 `this`，而 AsyncCAN 未完成的读通过 shared_from_this 保留接收器，因此离开探测函数并不会自动停止接收。后续 CAN 帧能继续写入已过期的 detector 标志位，破坏复用的栈内存。

在固定 ugv_sdk HEAD `f2704eacdc90357078cd93ec60aae08bb4baab35` 上增加最小补丁：初始化标志位，探测器析构时显式 Close 接收器；真实 AsyncCAN::Close 会停止 io 并 join 接收线程，再允许 detector 标志位销毁。未改变底盘控制协议、运动命令、遥控优先级或导航门禁。启动脚本从最后 64 KiB 日志中提取子进程崩溃行，正常退出清理日志不再掩盖前面的错误；仍然停止本次自有组件，不自动重试硬件。

验证结果：

- 用真实 protocol_detector 源码配离线接收器 fixture，不打开 CAN、不启动 ROS/实车驱动。固定原始版本作为负对照，AddressSanitizer 报 stack-use-after-scope；补丁版本接收器关闭，后续投递不访问旧对象。
- 在临时锁定 checkout 上验证补丁 check 不写文件、重复应用幂等、SHA 保持不变。
- 启动脚本回归模拟 launch 返回 0、底盘子进程 -6、后续 25 行正常关闭日志，确认直接显示底盘错误。
- **30 passed in 16.52 s**，涵盖 native 生命周期、补丁重放、组合启动、隔离 ROS readiness、组件所有权与前台时钟入口。
- SDK 构建/安装、`colcon build --packages-select agt_bunker_base --symlink-install` 通过。没有整工作区清理。更新前 SDK 静态库、底盘可执行文件与源码已备份。
- 首次部分 Python 测试未加载 ROS overlay，两个包查找测试失败；加载已有 ros_exec 环境后通过。最终全套在 localhost DDS 域 183 完成。

源证据：工作区 `runs/bunker_startup_crash_20261025/` 的 `before/`、`core.43960`、`core_info.txt`、`backtrace.txt`、`inspect.txt`、SDK/driver build 日志和 `regression_final.log`。core 只在本地保存，未上传。依赖审计无 SHA 变化/重复包；原有 `external/bunker_ros2`、`external/fast_lio2_mapping` 本地修改原样保留，整体审计仍因这两项返回非零，不能宣称全仓 clean。

交付保存于 `patches/ugv_protocol_lifetime.patch` 与带 before/after SHA 的 manifest，init_workspace/build_humble 及 audit 已接入。初始化不会丢失该补丁；升级后需重建 SDK 并重新链接底盘包。当前工作区两步已完成。

本轮没有启动或停止实车底盘，没有发运动/重定位请求。实际 `field_start nav` 重启验证待操作者执行；此修复不宣称已解决所有底盘故障或位置相关的 BBS/GICP 失败。
