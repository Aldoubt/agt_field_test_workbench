# 导航请求、动作状态与停止回执

本协议扩展现有 `/api/control/{goal,route,distance,stop}` 和 `/api/live`，保留旧接口字段。
地图身份、定位新鲜度、物理控制权限、生命周期、端点检查、相机健康检查及运动门限保持原逻辑。
HTTP 返回不代表导航完成；完成必须由对应 ROS Action 的结果确认。

## 请求与身份

客户端每次明确操作生成一个新的 UUID 字符串作为 `request_id`，发送失败后不自动重发。
同一桥接进程内，相同请求 ID 和相同载荷只返回已有回执；相同 ID 配不同载荷会被拒绝。
未提供 ID 的旧客户端仍可调用，桥接为其生成 ID，但客户端无法可靠关联一个未收到回执的请求。

```json
{
  "request_id": "client-generated-uuid",
  "robot": "bunker",
  "map_id": "map-id",
  "route_id": "route-id"
}
```

回执保留顶层旧 `state`：运动提交为 `REQUESTED`，停止提交为 `STOP_REQUESTED`。
新增的 **`action.state` 才是当前动作阶段**。顶层还有
`action_id`、`request_id`、`bridge_id`、`seq`、`terminal`，与嵌套 `action` 一致。

`action` 包含以下字段：

| 字段 | 含义 |
|---|---|
| `bridge_id` | 本次桥接进程的唯一身份，重启会变化 |
| `action_id` / `request_id` | 服务端动作身份 / 客户端请求身份 |
| `kind` | `goal`、`route`、`distance` 或 `stop` |
| `state` / `terminal` | 当前阶段 / 是否已确认终态 |
| `seq` | 同一桥接内严格递增的更新序号 |
| `created_at` / `updated_at` | UTC ISO 时间 |
| `reason` | 当前原因或失败说明 |
| `outcome_unknown` | 已开始发送，但结果不可观测时为 `true` |
| `map_id` / `robot` / `route_id` | 原请求中存在时保留，用于显示上下文 |

运动状态为 `REQUESTED → ACCEPTED → RUNNING → SUCCEEDED/FAILED`；没有反馈的动作可由
`ACCEPTED` 直接到终态。明确预检失败、Action 拒收为 `REJECTED`；停止过程为
`CANCEL_REQUESTED → CANCELED`。取消与完成竞争时，服务器真实返回成功，也可能得到 `SUCCEEDED`。
终态不可被旧反馈、旧受理回调或其他请求的结果改写。

`status=4` 的 ROS 传输成功仍会检查任务结果：巡检 `success=false`、非零 `error_code`，
或者 FollowWaypoints 存在 `missed_waypoints`，都标记为 `FAILED`。

## 只读状态与 UI 锁

`GET /api/live` 新增：

```text
action_status = { bridge_id, sequence, busy, current, last, recent[] }
```

`current` 是当前未完成的运动动作；`last` 是最近更新的记录；`recent` 按更新时间倒序最多 32 条。
原有 `navigation_activity.pending/active` 和 `goal_result.status/result` 保留，另外提供
`navigation_activity.busy`，并为 `goal_result` 增加动作身份及其 `action`。

客户端提交时立即锁定该请求，收到回执后绑定 `(bridge_id, action_id, seq)`。之后只接受相同
桥接及动作身份、且序号不旧于回执的状态更新。仅匹配的终态才能结束该次等待；不能用短暂的
`pending=false/active=false`，或上一任务遗留的 `goal_result`，判定当前请求已完成。
另一个动作被拒绝也不会清除当前正在执行或取消中的动作。

发送、受理响应或结果响应异常并不能证明目标未执行。此时动作保持 `CANCEL_REQUESTED`，
`outcome_unknown=true`，桥接继续拒绝新的运动请求。客户端显示结果未知，不自动重发。
重启桥接会生成新的 `bridge_id`，不构成旧动作已经结束的证据。

## 停止

停止使用自己的新 `request_id`。如果客户端正在等待一个未知结果，可同时发送
`target_request_id` 指向那个运动请求。桥接会保存取消标记，原运动请求即使随后才到达，也会
被拒绝执行；这避免 HTTP 超时后出现“先点停止，旧请求却晚到执行”的顺序问题。

停止首先执行既有零速、deadman 关闭、取消及 FW-mini 禁能逻辑：

- 尚未发送的请求由 epoch 失效，标记 `CANCELED`。
- 已发送或受理的请求保持 `CANCEL_REQUESTED`，等待真实 Action 终态。
- 停止自己的动作在等待时为 `STOP_REQUESTED`。所有目标有结果后为 `DISPATCHED`；没有未决目标
  时可以立即达到此终态。客户端仍从随后 `/api/live` 中的对应停止 ID 确认。

`DISPATCHED` 表示零速已发布、此前动作请求已解决，**不表示已测得车体静止**。取消服务的回执
也不等于 Action 已取消。坏的可选 `target_request_id` 会被忽略并记录 `correlation_warning`，
不会阻断停止本体。已有运动请求结果完全不可观测时，停止状态也会保持等待；不得伪造取消成功。

## 错误与兼容

HTTP 400 保留 `error` 字符串，并在已分配身份时附 `action` 对象。结果未知还会附
`outcome_unknown=true` 和 `request_id`。Unix RPC 的 `ControlError` 是 `ValueError` 子类，
既有异常捕获仍有效。桥接队列等待超时保留已有请求身份，且不会重试动作。

完整记录保留 128 条及所有未完成动作；终态摘要和请求去重标记在桥接生命周期内保留。
4096 个唯一请求后新运动请求会明确拒绝，停止仍可使用；不会通过删除去重标记重新执行旧请求。

## 验证

```bash
python3 -m pytest -q tests/test_action_status.py tests/test_bridge_actions.py \
  tests/test_control_error_protocol.py
```

50 项离线测试通过：真实状态机、生产 Bridge 方法与确定性 Action Future、真实 Unix socket RPC，
以及生产 HTTP 路由错误协议。覆盖二次预检、重复请求、迟到受理、取消竞争、无法观测的结果、
普通路线及拍照路线。测试注入 ROS 消息与客户端，不连接 ROS 图、不控制机器人；真实 DDS/硬件
流程仍需在部署环境验证。
