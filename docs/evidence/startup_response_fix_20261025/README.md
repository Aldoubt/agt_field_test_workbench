# 距离模式启动响应超时与任务被取消

现场记录：`runs/field_mode_2026-10-25T113119_759742+0000/session.json`，核心进程 `20261025T113120Z-f97b7325`，执行器 `20261025T113132Z-e02c27a6`。

执行器日志记录 `failed to send response (timeout): client will not receive response`。启动器对 `/mission_runtime/get_parameters` 的单次请求等待 45 秒后退出。用户另一个终端提交距离任务时参数查询成功，HOME_START 正常结束，D5m 刚开始即遭清理取消。不能据此认定执行器死锁或确定 DDS 内部的具体故障原因。

同一轮 bt_navigator 日志 `/home/yangxuan/.ros/log/bt_navigator_10179_1792927882948.log`：

| ROS 时间秒 | 事件 |
|---|---|
| 1792927936.6708 | HOME_START 开始，随后成功 |
| 1792927938.622743352 | 开始从 (18.33, -9.20) 导航至 (23.31, -9.68) |
| 1792927941.024739925 | 客户端请求取消目标 |
| 1792927941.024939926 | 目标已取消 |
| 1792927941.291663260 | 核心进程收到 SIGINT/SIGTERM |

D5m 进入 Nav2 到被取消约 2.402 秒。该时段没有完整的速度链路采样，不能据此排除遥控状态、底盘接收或其他独立问题，也不宣称距离实车测试通过。

修复：只对幂等的参数读取增加有界重试（发现 8 秒、响应 2 秒、最多 3 次），复用同一个发现成功的客户端，移除过期请求，清理客户端。保留会话进程健康检查。生命周期激活、重定位 Trigger、参数写入均维持原单次调用。

组合入口发布具有 PID 和进程启动时间的本地就绪状态。距离 run/home 与拍照 start 在准备或退出期间不提交任务；原有分开启动不依赖组合入口，历史进程状态文件不阻断旧工作流。未放宽定位新鲜度或到达判定，未改底盘驱动、FAST-LIO 或依赖版本。

回归在 `ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1` 内执行。包括真实 DDS 的首次响应丢失与迟到、持续丢失、无效响应、组件退出；组合入口响应丢失后就绪/持续失败后清理；准备阶段拒绝提交；真实 mission_runtime 启动参数和 Action 发现；已有分开启动、TTY 时钟助手、距离任务与拍照流程。测试不启动硬件、不发送实车目标、不调整现场时钟服务。

结果：**81 passed in 22.20s**，详见同目录 `regression.txt`、`regression.xml`。从 Workbench 仓库目录复现：

```bash
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD" \
  bash scripts/ros_exec.sh python3 -m pytest -q \
  tests/test_ros_parameters.py tests/test_mode_state.py tests/test_field_distance_ros.py \
  tests/test_field_start_ros.py tests/test_field_runtime_process.py tests/test_field_start.py \
  tests/test_field_start_tty.py tests/test_field_clock.py tests/test_field_tasks.py tests/test_field_photo.py
```

实车下一步由操作者启动 `./field_start distance`，等最终“导航已激活”提示，再另开终端执行 `./field_distance run 5`。原有任务文件无需重建。
