# FW-mini 4T4D / 横移档位控制交付

日期：2026-10-23（北京时间）。起点为干净 `main` 的 `76ada2d`，已包含用户给出的 `61b2f9302b2d689ee39808ded03ee4d93f357c01`。本轮只修改控制链路、依赖补丁部署和测试；没有开启自主导航。

## A. 软件交付

### 协议和修改位置

采用用户指定的《FW-mini 用户使用手册 V1.0.4》第 15 页 **CtrlCmd**：0 Disable、1 驻车、2 空档、6 4T4D（vx+wz）、8 横移（vx+vy）。本轮不使用 SteeringCtrlCmd 的 5/7 编码。CAN 为 **500000 bit/s**，指令 ID `0x18C4D1D0`，反馈 ID `0x18C4D1EF`；厂家 SocketCAN 代码中的 `0x98…` 包含扩展帧标志。

保持工作区依赖的锁定提交，通过[控制补丁清单](../patches/control_manifest.json)精确应用：

- [fwmini_ctrl_cmd.patch](../patches/fwmini_ctrl_cmd.patch)：`fwmini_chassis_adapter` 的 core、node、配置、测试；厂家驱动仅新增独立 IO 反馈发布。
- [fwmini_guard.patch](../patches/fwmini_guard.patch)：V3 `agt_base_control/cmd_vel_guard.py` 增加独立 FW-mini 运动学配置，Bunker 默认行为保持非全向。
- `agt_field_test_bringup/launch/field_test.launch.py`：FW-mini 专用 guard 参数和受控输出，Bunker 显式使用 bunker 配置。
- `scripts/apply_control_integration.py`、初始化/构建/审计脚本：检查原始和修改后 SHA-256，拒绝覆盖其他本地修改；不改变依赖 HEAD，不创建分支。干净克隆、重复应用及保留现场修改均有测试。

初始化及构建：

```bash
# 在工作区 src/agt_field_test_workbench 内执行
bash scripts/init_workspace.sh
bash scripts/build_humble.sh
```

现有工作区也可先运行 `python3 scripts/apply_control_integration.py`，再在 Humble overlay 下针对 `yhs_can_control fwmini_chassis_adapter agt_base_control agt_field_test_bringup` 执行 colcon build。模板不会覆盖工控机已有 config；已有 `config/robots/fwmini.yaml` 的 CAN 值应由操作者核对为 500000。

### 状态与命令链路

```text
DISABLED (gear=0, zero)
   └─显式选模式─> STOPPING (gear=0, zero，连续停止反馈)
                       └─> SWITCHING (gear=6/8, zero，连续匹配反馈)
                                  └─> READY (gear=6/8, zero)
                                           └─显式 enable─> ACTIVE
ACTIVE ──选新模式──> STOPPING
任意安全异常 ──> FAULT (gear=0, zero，保留首次故障原因)
任意状态 ──disable──> DISABLED (gear=0, zero)
```

READY 之前 `/fwmini/enable_motion true` 必须失败；READY 时还要求 guard 已确认当前 epoch 的新鲜零速心跳，缺失则保持 READY 并拒绝启用。模式请求服务成功只表示接受请求，完成结果以状态中的 `last_mode_switch_result=SUCCESS` 和 READY 为准。切换失败、禁用或故障后，需重新选模式、重新确认，再启用；反馈恢复本身不恢复运动。

默认检查：停车阈值 |vx|、|vy| ≤ 0.01 m/s，|wz| ≤ 0.02 rad/s；至少 3 个连续反馈且跨度 ≥ 0.06 s，反馈间隔必须小于 0.2 s；总切换超时 3 s。CtrlFb 和独立 IoFb 均须在 0.2 s 内新鲜。CtrlFb ROS 时间戳还须递增且年龄 < 0.2 s，拒绝重复、旧和未来时间戳。停车与档位确认分别使用新的反馈序列，不能用请求前的停车历史直接进入运动档。

实体急停 `io_fb_estop=true`、遥控接管 `io_fb_joypad_ctrl=true`、反馈过期、档位不符、NaN/Inf、非法速度轴、上游指令超时、guard 丢失或外部停止均清零并进入 FAULT。IO 位含义、比例和实体响应仍需按下节进行实车核对。

```text
/agt/hmi/cmd_vel (Twist，仅显式 manual 来源)
    → agt_cmd_vel_guard [kinematic_profile=fwmini]
    → /fwmini/cmd_vel_guarded (TwistStamped，绑定本次启用 epoch)
    → fwmini_chassis_adapter
    → /ctrl_cmd (yhs_can_interfaces/CtrlCmd)
    → 厂家 yhs_can_control（仅实机入口启动）
```

FW-mini 适配器不再订阅 `/cmd_vel`。6 档拒绝非零 vy，8 档拒绝非零 wz；所有模式拒绝 linear.z/angular.x/angular.y 和非有限值，不能静默丢轴。guard 做对应轴的限幅与加速度限制，适配器独立复核。默认低速上限 vx=0.05、vy=0.03 m/s、wz=0.15 rad/s；guard 倒车上限 0.03 m/s。

每次切换、确认、启用、禁用、故障均使 epoch 改变；节点重启使用新的会话种子。guard 清空旧目标，适配器拒绝旧 epoch，即使旧消息仍在传输也不能重放。原始手动 Twist 的新鲜性以接收时刻判断；受控消息另检查时间戳。此话题隔离与 ROS 图检查不是 ROS 网络访问控制。

Bunker 继续输出普通 Twist 至 `/mux/cmd_vel`，linear.y 始终为零，原有定位失效停车门控继续生效。FW-mini 配置只允许受监督手动来源，没有开放 Nav2。

### 50 Hz、CAN 与发布权

正常 CtrlCmd 由 20 ms wall timer 发布；状态改变和安全故障立即清空内部速度，并优先安排零速。所有 CtrlCmd 发布都保持至少 10 ms 间隔，避免连续状态变化产生过密 CAN 请求；正常退出的三个零速帧相隔 20 ms。单位边界仍为 ROS m/s、rad/s 转厂家 m/s、deg/s，厂家再打包 CAN。厂家 `ctrl_cmd_callback` 的速度转换、AliveCounter、BCC 和字节打包逐字保持不变，并有对照锁定提交的测试。

厂家原复合反馈会缓存旧 IoFb，不能以 CtrlFb 时间戳证明 IO 新鲜。本轮只在驱动通过 IO BCC 校验后增加 `/fwmini/io_fb`（IoFb）发布，适配器独立计龄；CAN 收发格式和校验没有改动。厂家接收代码对 y 速度使用 `/100`，发送代码使用 `*1000`；此次保留该行为，其实际量纲需通过正式手册和实车反馈核对，未凭推测改比例。

同主机同 ROS domain 用进程锁阻止第二个适配器建立 CtrlCmd publisher。运行中发现 `/ctrl_cmd` 存在其他 publisher，适配器发布故障零速后退出并释放发布权。跨主机 DDS 图发现有延迟，不能把该检查视为物理互锁；现场必须先检查 publisher 唯一性，并禁止其他控制节点。停止唯一适配器后是否真正制动取决于底盘硬件超时，仍需实机验收。

### 服务和诊断

| 名称 | 类型 / 作用 |
| --- | --- |
| `/fwmini/select_four_wheel_steer` | std_srvs/Trigger，请求 6 档 |
| `/fwmini/select_lateral` | std_srvs/Trigger，请求 8 档 |
| `/fwmini/enable_motion` | std_srvs/SetBool；true 仅 READY 成功，false 禁用 |
| `/fwmini/stop` | std_srvs/Trigger，外部停止并锁定 FAULT |
| `/fwmini/get_control_state` | std_srvs/Trigger，返回 JSON 状态快照 |
| `/fwmini/control_status` | std_msgs/String JSON 持续诊断 |
| `/fwmini/external_stop` | std_msgs/Bool；true 锁定外部停止，false 只清外部条件，不自动恢复 |

诊断包括 state、requested_mode、target_gear、actual_gear、output_gear、feedback_age_sec、io_feedback_age_sec、motion_allowed、command_mps_radps、epoch、reason、last_mode_switch_result，以及 guard_epoch_seen、guard_feedback_age_sec。反馈从未收到时年龄为 -1。首次故障原因保持到显式恢复，后续停止消息不能覆盖根因。

架轮零速阶段的查询/选档示例（选择一项模式后等待 READY，不会自动启用）：

```bash
ros2 service call /fwmini/select_four_wheel_steer std_srvs/srv/Trigger '{}'
# 横移模式使用 /fwmini/select_lateral，同为 Trigger
ros2 service call /fwmini/get_control_state std_srvs/srv/Trigger '{}'
ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool '{data: false}'
```

`enable_motion true` 只用于下文完成准备的受监督运动验收阶段，之后也必须发送新的手动速度指令。

外部停止订阅使用 reliable/transient_local；现场停止发布端应持有同等 QoS，并维持有效连接。例如 `ros2 topic pub /fwmini/external_stop std_msgs/msg/Bool '{data: true}' --qos-durability transient_local -r 10`。释放该条件后仍需重新选模式和启用。

### 无 CAN 软件验收

命令均在真实 Ubuntu 22.04 / ROS 2 Humble 容器执行，无 CAN 设备，无厂家 CAN 驱动进程，无实车运动：

```bash
# 工作区根目录；控制测试使用独立 ROS domain
ROS_DOMAIN_ID=216 bash src/agt_field_test_workbench/scripts/ros_exec.sh \
  python3 src/agt_field_test_workbench/tests/humble_fwmini_control.py

# 标准 CTest/colcon 入口也运行同一 ROS 套件和 C++ core
colcon test --base-paths src --packages-select fwmini_chassis_adapter
colcon test-result --test-result-base build/fwmini_chassis_adapter --verbose
```

结果和实际消息序列见 [证据目录](evidence/fwmini_control_20261023/README.md)。模拟厂家节点实际发布 CtrlFb/IoFb，观察真实适配器 CtrlCmd；测试没有替换适配器或 guard 的执行逻辑。

## B. 实机验收步骤（全部 NOT_RUN）

下列均尚无实机记录，不能用上述软件结果签收：

1. **准备与接管（NOT_RUN）**：记录底盘/控制器固件、手册版本、传感器和日期；在安全区域核验实体急停、遥控器优先级、接管操作、厂家自主/遥控模式含义。明确 0/1/2 档的物理行为；gear=0 零速不是已证明的制动性能。未经确认不发送非零速度。
2. **CAN 只读反馈（NOT_RUN）**：核对 SocketCAN 500000 bit/s 和扩展帧，使用 candump 记录 `0x18C4D1EF`、IO 反馈及频率/BCC/AliveCounter。核对 CtrlFb xyz 单位、角速度 deg/s、IO 急停与遥控位极性。特别核对厂家 y 反馈比例；不以软件解码值代替实测单位。
3. **架轮、0 档与发布权（NOT_RUN）**：可靠架轮固定、人员远离轮系，急停和遥控接管有效。启动本工作区唯一控制链路；`ros2 topic info /ctrl_cmd --verbose` 必须仅一个 publisher。检查启动 gear=0、全部速度严格为零，没有自动进 6/8 档。录制 `/ctrl_cmd /chassis_info_fb /fwmini/io_fb /fwmini/control_status /fwmini/cmd_vel_guarded /agt/hmi/cmd_vel`。
4. **架轮零速选档（NOT_RUN）**：只调用选模式服务，不启用运动；观察先 STOPPING，再零速发 6/8，反馈连续匹配才 READY。测试目标档不符、断反馈、IO 丢失和重复时间戳时不允许启用。确认没有旧速度恢复。记录完整消息、CAN 原帧和状态转换时间。
5. **受监督低速直线/转向（NOT_RUN）**：完成前述步骤并确认安全后，落地清空场地，配置实测几何，显式选 4T4D，等待 READY，人工调用 enable。新发送低速 vx、再测试 vx+wz；先从低于软件上限的值开始。验证制动距离、响应时间、轮向与速度正负号，不能仅看消息为零。
6. **受监督低速横移（NOT_RUN）**：先停车并由反馈确认，再选 lateral，等待 READY，重新 enable，发送新的低速 vx/vy。验证横向正负、轮向、车体滑移与实际速度；禁止 wz。切回 4T4D 时重复完整停车确认流程，不允许保留横移命令。
7. **故障与接管（NOT_RUN）**：逐项执行命令源停止、guard/适配器退出、外部 stop/disable、CAN/IO 反馈中断、遥控接管和实体急停；记录实际停车时间和距离、控制权交接、停止后旧指令不可恢复。硬件超时和制动效果必须实测，不接受仅消息序列归零。
8. **归档签收（NOT_RUN）**：保存操作者、固件、参数、bag/CAN 日志 SHA-256、每项观测与失败情况。只有有记录的项目才升级 PASS；自主导航继续关闭。
