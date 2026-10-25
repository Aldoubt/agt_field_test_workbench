# 组合启动现场修复：终端认证与空字符串参数

2026-10-25 现场第一次 distance 启动在 sudo 认证阶段退出，日志为 runs/20261025T112034Z-25035db7/process.log。组合入口将时钟脚本放在 Jobs 的 setsid 子会话中，没有控制终端，sudo 无法询问密码；雷达/底盘 launch 尚未执行。修复为先检查旧组件，再在当前前台终端进入原时钟保持脚本，认证成功后重新进入组合入口；后台导航组件不再运行 sudo。原时钟暂停/恢复逻辑不变。伪终端测试验证了原失败、前台成功、后台组件仍隔离，以及 Ctrl+C 先等子程序退出再恢复服务；系统服务和 sudo 都是 mock。

第二次启动已通过新鲜 odom、全局定位及 Nav2 激活，在启动距离执行器时失败：Humble 不接受 CLI 参数 `nav_behavior_tree:=`。修复为 argv 中传入 `nav_behavior_tree:=""`，保留显式空字符串与原精度行为树语义。另增加失败时打印日志末尾，减少二次手动查日志。

28 项相关回归通过，见 tests.xml。新增真实 Humble 子进程测试使用生产距离执行器 argv，在 ROS_DOMAIN_ID=183 / ROS_LOCALHOST_ONLY=1 中启动真实 mission_runtime，读取 GetParameters 确认 nav_behavior_tree 为空、recenter_after_capture=false，并发现任务 Action 服务端；没有发送任务目标。原测试只验证启动计划和模拟服务，未覆盖真实 CLI 参数解析，此次补齐。

本轮未操作现场底盘、云台或时钟服务，没有自动重试现场组合启动。脚本直接生效，无需 colcon 重编译。修复后由操作者保持停车，重新执行 `./field_start distance`。既有分开启动与 `./field_photo start` 保持可用，导航阶段标签不移动。
