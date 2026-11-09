---
name: flight-control
description: 把已生成的任务航迹组装成每架无人机的原始 WGS84 动作序列，用 dry-run 做格式校验和坐标转换检查，并通过中央服务完成下发与运行控制。用于动作组装、校验、中央提交及运行处置。
---

# 动作序列与中央下发

## 动作

每条动作：`{"cmd":"<动作>","task_id":"<task_id>","step_id":<序号>,"params":{…}}`。同一序列的 `task_id` 相同，`step_id` 从 1 起按顺序加 1。

| 动作 | 必须写的 params | 只在用户明确指定时写 |
|---|---|---|
| `arm` | `{}` | — |
| `takeoff` | `altitude`（该机 `flight_alt`） | `speed` |
| `follow_path` | `waypoints`、`"skip_unreachable": false` | `speed` |
| `fuel_explore` | `box_min`、`box_max`、`timeout_sec` | `max_vel` |
| `patrol_move` | `waypoints`、`"skip_unreachable": false` | `speed` |
| `land` | `{}` | — |

- 其他参数由校验脚本按默认值补齐（速度 1.2 m/s，FUEL 速度 0.6 m/s），不要写进序列。
- 用户指定速度时，要同时写 `takeoff`、`follow_path`、`patrol_move` 的 `speed` 和 `fuel_explore` 的 `max_vel`。
- 航点格式 `{"longitude":…,"latitude":…,"altitude":…}`，可带工具返回的 `pass_through`；搜索框格式 `[经度,纬度,高度]`。

## 由任务航迹组装

对任务航迹 `planning_routes.py` 的 `build` 结果中的每个 `assignments[i]`（完整数据在返回文件的 `data` 中）：

1. `task_id` = `<task_key>-<uav_id>`，例如 `T1021153012a3f2-UAV-003`。
2. 依次写：`arm` → `takeoff`（`altitude` = `flight_alt`）→ 按 `steps` 顺序转换 → `land`：
   - `transit` → `follow_path`，`waypoints` 原样复制；
   - `search` → `fuel_explore`，复制 `box_min`、`box_max`、`timeout_sec`；
   - `patrol` → `patrol_move`，`waypoints` 原样复制，重复 `laps` 次（每圈一条动作）。
3. 航点和搜索框逐字复制，不取整、不删点、不调整顺序。

用户要求无目标探索/巡逻完成后原地悬停时，仍保留 `build` 给出的返航转场和 `land`，作为中央登记需要的备用收尾段；另在方案中明确 `policies.on_search_complete="HOLD"`。中央把作业段和返航段分开，由策略或后续返航指令决定是否执行收尾，不是把整个 batch 直接连续发布。离线只验证格式与转换，不能称已经验证悬停执行或无限等待续航；不能删除返航段来“实现HOLD”。

一架机一个搜索作业的序列长这样（航点已省略）：

```json
[
  {"cmd":"arm","task_id":"T1021153012a3f2-UAV-003","step_id":1,"params":{}},
  {"cmd":"takeoff","task_id":"T1021153012a3f2-UAV-003","step_id":2,"params":{"altitude":1.2}},
  {"cmd":"follow_path","task_id":"T1021153012a3f2-UAV-003","step_id":3,"params":{"waypoints":[…],"skip_unreachable":false}},
  {"cmd":"fuel_explore","task_id":"T1021153012a3f2-UAV-003","step_id":4,"params":{"box_min":[118.727366,31.174749,0.3],"box_max":[118.727492,31.174848,1.7],"timeout_sec":1200.0}},
  {"cmd":"follow_path","task_id":"T1021153012a3f2-UAV-003","step_id":5,"params":{"waypoints":[…],"skip_unreachable":false}},
  {"cmd":"land","task_id":"T1021153012a3f2-UAV-003","step_id":6,"params":{}}
]
```

## 只规划：dry-run 校验

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/flight-control/scripts/submit_queue_batch.py" \
  --json '<动作序列 JSON 数组>' --origin-lat <origin.latitude> --origin-lon <origin.longitude> --dry-run
```

`--origin-lat/--origin-lon` 取该机航迹结果里的 `origin`（起飞点），不能用任务中心或某个航点代替。

| 结果 | 处理 |
|---|---|
| `status=OK`，`submitted=false` | 校验通过 |
| `INVALID_ARGUMENT` | 按 `message` 修正序列后重试一次，仍失败就报告 |
| `ERROR` | 报告错误并停止 |

返回中的 `batch` 是已转换为该机 MAVROS 本地坐标、补齐默认参数的 ROS 飞控动作序列：只规划时把它原样作为最终输出；它不能再交给 register-plan 或任何转换命令（不能二次转换）。单项“校验动作序列”请求也用这条命令。

通用 dry-run 不执行中央任务的作业/收尾校验：纯转场数组、删掉返航段的数组可能通过格式校验，但不能据此称当前中央任务支持它。完整任务还须满足本技能末尾的中央计划规则；单项动作校验无需强加搜索或巡逻。

## 执行：中央下发四步

用户确认了具体任务确认卡后才做这四步；确认边界见 AGENTS.md（故障补做不需要确认）。

所有中央命令的写法：

```bash
python3 "$MIGRATION_ROOT/scripts/mission_client.py" <操作> --agent-session-id "$NANOBOT_SESSION_ID" --json '<JSON>'
```

`status` 不带 `--json`：`python3 "$MIGRATION_ROOT/scripts/mission_client.py" status --agent-session-id "$NANOBOT_SESSION_ID"`。

`request_id` 只有 register-plan、submit 和运行控制需要；authorize、preview 没有 `request_id`。

**1. register-plan**（每架机一次，可并行）：

```json
{"request_id":"reg-<task_key>-<uav_id>","uav_id":"<uav_id>","batch":[原始 WGS84 动作序列],
 "perception":{"query_id":"<该机 query_id>","session_id":"<该机 session_id>"}}
```

- `batch` 逐字复制确认卡中该机的序列；故障补做没有确认卡，用组装结果。
- 没有目标时省略 `perception`；有目标时取 `status` 返回的 `fleet.<uav_id>.perception`，两个值必须完全一致。
- 注册时会用与 dry-run 相同的校验器检查序列，执行模式下不必另做 dry-run。成功返回 `plan_ref`。

**2. authorize**（整个任务一次）：

```json
{"user_intent":"<原指令>；用户确认：<确认原话>",
 "spec":{"participants":[{"uav_id":"UAV-003","plan_ref":{"id":"plan-…","revision":1}}],
         "targets":[目标清单，没有目标时为 []],
         "policies":{"land_on_return":true},
         "authorization_ref":"pending-authorization"}}
```

`user_intent` 写用户的原指令和确认的原话，中央会把它和 spec 摘要一起存进授权记录；故障补做写“<原指令>；故障补做：<故障机> 未完成的 <作业名>”。取返回结果中的 `spec` 字段（已带真实 `authorization_ref`），后面两步原样使用这份 `spec`。用户要求“发现后跟踪”时，`policies` 改为：

```json
{"land_on_return":true,"on_first_confirmed":"APPROACH_AND_TRACK","tracking_target_ids":["<target_id>"],
 "approach_profile_ref":{"id":"approach-default","revision":7},"tracking_profile_ref":{"id":"tracking-default","revision":1}}
```

两个 profile 的 id 和 revision 以 `status.target_action_profiles` 的返回为准。跟踪时长、距离由 profile 固定，用户另指定时说明暂不支持修改。用户明确要求任务结束后不返航时，在 `policies` 里加 `"on_search_complete":"HOLD"`（作业完成后原地悬停，等待后续指令）。

**3. preview**：`{"spec":<authorize 返回的 spec>}`，返回 `preview_id`（60 s 内有效，只能用一次），拿到后立即 submit。

**4. submit**：

```bash
python3 "$MIGRATION_ROOT/scripts/mission_client.py" submit --agent-session-id "$NANOBOT_SESSION_ID" \
  --json '{"request_id":"submit-<task_key>","preview_id":"<preview_id>","agent_session_id":"'"$NANOBOT_SESSION_ID"'","spec":<同一份 spec>}'
```

返回 `mission_id` 和 `status=ACCEPTED`，表示任务已创建，不表示已起飞或已完成。

## 失败处理

| 错误码 | 处理 |
|---|---|
| `INVALID_PLAN`、`INVALID_INPUT` | 按 `message` 修正后重试一次；不要原样重发。用户已确认的动作序列不自行修改：报告原因，重新规划并重新出卡 |
| `UAV_UNAVAILABLE`、`UAV_BUSY` | 该机不在线或被占用：停止并报告 |
| `PERCEPTION_UNAVAILABLE` | 感知引用与当前会话不一致：重新读 `status` 的 `fleet.<uav_id>.perception` 再注册 |
| `EXECUTION_DISABLED`、`ADAPTER_UNAVAILABLE`、`SERVICE_RECOVERING` | 服务当前不能执行：报告并停止 |
| `PREVIEW_EXPIRED`、`PREVIEW_CONSUMED` | 用同一份 `spec` 重新 preview，再 submit |
| `PREVIEW_CHANGED` | submit 的 `spec` 必须与 preview 完全相同 |
| `IDEMPOTENCY_CONFLICT` | 同一 `request_id` 用了不同内容：恢复原内容，或确认是新动作后换新 ID |
| 网络超时、结果不明 | 先 `get`/`events` 确认；需要重试时沿用同一 `request_id` |

## 运行控制

暂停、继续、返航、取消、跟踪控制、待决策处理和故障诊断，读 [references/runtime-control.md](references/runtime-control.md)。

## 注意事项

- 中央接管的机只接受中央控制：不要用 `submit_queue_batch.py` 去掉 `--dry-run` 发布，也不要用 `fleet_dispatch.py` 或直接发布 ROS 动作话题。
- `register-plan` 只接受原始 WGS84 序列；dry-run 输出（带 `_coordinates_already_mavros`）不能再提交。
- 中央计划的规则：以 `arm`、`takeoff` 开头；至少一个 `fuel_explore` 或 `patrol_move`；以 `follow_path`（可再加 `land`）结尾；`fuel_explore` 必须写 `timeout_sec`；不能用 `repeat`、`loop_forever`，多圈要逐圈展开。
- `user_intent` 中如有英文单引号要去掉，否则会破坏 `--json '…'` 的引号。
- 计划注册成功、已授权、已提交、已执行是四件不同的事，按实际返回分别报告。