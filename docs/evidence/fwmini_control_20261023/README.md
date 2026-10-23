# 无 CAN 的 Humble 控制验收证据

2026-10-23（北京时间）；Ubuntu 22.04 / ROS 2 Humble Docker。无 CAN 设备，未启动厂家 CAN 驱动；CtrlFb/IoFb 由测试节点模拟，guard 和适配器均为实际安装的 ROS 节点。

| 项目 | 结果 |
| --- | --- |
| yhs_can_control、fwmini_chassis_adapter、agt_base_control、agt_field_test_bringup 编译 | PASS |
| C++ core：启动禁用、6/8 档、停车/档位连续确认、超时、轴限制、epoch、急停/控制权和单位转换 | PASS |
| 真实 Humble 控制集成用例 | 9/9 PASS |
| 标准 colcon/CTest（core + ROS 套件） | 2/2 PASS |
| 工作区 Python/API/部署补丁测试 | 13/13 PASS |
| Bunker 原有 guard 失效停车/来源切换验收脚本 | PASS |
| 实车制动、急停、遥控接管、CAN 帧和低速运动 | NOT_RUN |

[机器结果](result.json)，[标准 CTest 日志](colcon_ctest.log)，[CtrlCmd 消息序列索引与 SHA-256](ctrl_cmd_sequences.json)。各测试的 adapter/guard 日志保留于同目录；第二发布者和重复实例导致的非零退出是预期拒绝路径。

`test_*.jsonl.gz` 是测试订阅 `/ctrl_cmd` 时逐条记录的原始消息序列。每行包含接收相对时间和最近的状态快照，字段映射：

- `gear` → `ctrl_cmd_gear`
- `vx_mps` → `ctrl_cmd_x_linear`
- `vy_mps` → `ctrl_cmd_y_linear`
- `wz_degps` → `ctrl_cmd_z_angular`

这些数值直接来自实际 CtrlCmd 回调，没有按预期值重写。解压示例：

```bash
gzip -dc test_modes_stop_before_shift_and_require_new_commands.jsonl.gz | head
```

九项 ROS 用例覆盖：启动与唯一发布权；停车后切换 6/8、零速及新命令要求；guard 非法轴/非有限值；适配器独立非法轴与旧 epoch 拒绝；档位不符和反馈断续；disable/stop/急停/遥控权/两类反馈过期/外部停止逐项原因；上游超时和重复时间戳；节点停止、重启及外部 publisher 冲突；Bunker 非全向约束。

复现命令见 [控制交付](../../fwmini_ctrl_cmd_delivery.md)。独立适配器校验用例持续发送新的上游零指令，避免上游 watchdog 抢先触发而掩盖被测拒绝路径。实机项目按交付文档 B 部分另行验收。

索引中的中位周期仅统计同会话、相邻的非零运动指令；记录的是 ROS 接收时间，CAN 总线时序仍需实机测量。
