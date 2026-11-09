# 运行控制

任务提交后的查询、控制、待决策处理和故障诊断。根据用户请求和中央最新事件选择适用操作；这里只给命令写法。

## 目录

- 查询：get 与 events
- 控制：control
- 待决策：decide
- 结果判定
- 故障诊断

## 查询：get 与 events

```bash
python3 "$MIGRATION_ROOT/scripts/mission_client.py" get --agent-session-id "$NANOBOT_SESSION_ID" --mission-id <mission_id>
python3 "$MIGRATION_ROOT/scripts/mission_client.py" events --agent-session-id "$NANOBOT_SESSION_ID" --mission-id <mission_id> --after-seq 0
```

`get` 快照里要看的字段：

| 字段 | 含义 |
|---|---|
| `revision` | 下一次 control/decide 的 `expected_revision` |
| `phase` | 任务阶段；SUCCEEDED、PARTIAL、FAILED、CANCELED 是终态。是否有待处理事项看 `decisions` 和 `exceptions`，不看 `phase` |
| `participants.<uav_id>.phase` | 各机阶段：SEARCHING 正在执行计划；HOLDING 原地悬停保持；RETURNING 返航中；DONE 已结束 |
| `participants.<uav_id>.containment` | `CONFIRMED_HOLD` 表示已确认停稳 |
| `participants.<uav_id>.region_jobs` | 各机作业及状态，以 `region_job_id` 为键；`step_id` 是该机动作序列的步骤号 |
| `targets[].state` | FOUND 已找到，PENDING 未找到 |
| `decisions` | `state=WAITING_DECISION` 的待决策，`reason` 是类型 |
| `exceptions` | `state=WAITING_DECISION` 的异常，含 `uav_id` 和 `reason` |
| `first_track_slot` | 跟踪中的机：`uav_id`；`state` 不是 ENDED、FAILED 时表示正在跟踪 |
| `assignment_options` | 待分配接手机时，各机是否可选（`eligible`）及距离（`distance_m`） |

`events` 是持久事件历史，谁在何处发现了哪个目标等细节从这里看；用最后一条的 `seq` 作为下次的 `--after-seq`。不要循环调用 `get` 等任务结束。

## 控制：control

```bash
python3 "$MIGRATION_ROOT/scripts/mission_client.py" control --agent-session-id "$NANOBOT_SESSION_ID" \
  --json '{"request_id":"<见下>","mission_id":"<mission_id>","expected_revision":<revision>,"operation":"return","scope":"uav","uav_id":"UAV-004"}'
```

整体控制把 `"scope":"uav","uav_id":"…"` 换成 `"scope":"mission"`。

| operation | 用途 | 说明 |
|---|---|---|
| `return` | 返航 | 整体或单机。对出现异常的机发送后，该机的异常自动结束 |
| `pause` | 暂停 | 只能暂停 `phase=SEARCHING` 的机，逐架 scope=uav 发送 |
| `resume` | 继续 | 恢复暂停时保存的进度；整体暂停用 scope=mission，单机用 scope=uav |
| `cancel` | 取消任务 | 只支持 scope=mission |
| `start_tracking` | 开始跟踪 | scope=uav，另加 `"target_id"`；目标须已确认且在授权跟踪清单内 |
| `stop_tracking` | 停止跟踪 | scope=uav；停止后中央自动恢复原搜索，并发出 `TRACKING_FINISHED` 待决策 |

- `request_id` 写成 `<operation>-<mission_id 末 8 位>-r<expected_revision>`，例如 `return-3f2a9c1d-r12`。结果不明需要重试时沿用同一个 ID。
- 多条 control 依次发送，每次用上一次返回的 `revision`，不要并行。
- 返回 `status=ACCEPTED` 只表示请求被接收；实际停住、开始返航要看 `get` 或 `events`。

## 待决策：decide

```bash
python3 "$MIGRATION_ROOT/scripts/mission_client.py" decide --agent-session-id "$NANOBOT_SESSION_ID" \
  --json '{"request_id":"decide-<decision_id 末 8 位>","mission_id":"<mission_id>","expected_revision":<revision>,"decision_id":"<decision_id>","operations":[…]}'
```

| 决策 | operations |
|---|---|
| `ALL_TARGETS_FOUND`，无机在跟踪 | `[{"operation":"return","scope":"mission"}]` |
| `ALL_TARGETS_FOUND`，UAV-002 在跟踪 | `[{"operation":"return","scope":"uav","uav_id":"UAV-002","after_tracking":true},{"operation":"return","scope":"uav","uav_id":"UAV-001"}]`（其余每架机各一条） |
| `TARGET_ASSIGNMENT_REQUIRED` | `[{"operation":"assign_target","scope":"uav","uav_id":"<选中的机>","target_id":"<决策中的 target_id>"}]` |
| `TRACKING_FINISHED`，还有目标未找到 | `[{"operation":"continue","scope":"uav","uav_id":"<跟踪机>"}]`（中央已自动恢复该机原搜索，continue 表示维持） |
| `TRACKING_FINISHED`，目标已全部找到 | `[{"operation":"return","scope":"uav","uav_id":"<跟踪机>"}]` |

- `after_tracking=true`：不打断正在进行的跟踪，跟踪结束后该机直接返航；届时只发通知，不再要求决策。
- 同一条 decide 里各操作的作用范围不能重叠，scope=mission 不能和 scope=uav 同时出现。
- 每个 `decision_id` 只能决策一次；返回 `DECISION_CLOSED` 时重新 `get`，不要重复提交。

## 结果判定

| 返回 | 含义与处理 |
|---|---|
| `REVISION_CONFLICT` | 快照已变：重新 `get`，按最新状态重新判断，再用新 `revision` 发送 |
| `UNSUPPORTED_INTERRUPT`、`CONTROL_CONFLICT` | 当前阶段不能执行该操作（如起降中暂停）：报告，不反复重试 |
| `NO_CHECKPOINT`、`STOP_UNCONFIRMED` | 没有可恢复的进度或停稳证据：报告，不重发起飞 |
| `MISSION_TERMINAL` | 任务已结束：报告终态 |
| `controls.<command_id>.status=APPLIED` | 控制条件已实际成立（如已开始返航）；到家、降落以后续事件为准 |

任务结果只认中央终态：SUCCEEDED 才是成功；PARTIAL 表示有作业没完成，要逐项说明；单步成功或最后的 land 成功都不代表整个任务成功。

## 故障诊断

用户问原因时，先读证据再下结论。可以给出建议，但重新起飞必须由用户明确下达新任务。

```bash
source "$ROS_SETUP" && source "$CATKIN_SETUP" && \
/usr/bin/python3 "$MIGRATION_ROOT/scripts/read_uav_status.py" --uav-id UAV-004 --kind execution --task-id <task_id>
python3 "$MIGRATION_ROOT/scripts/execution_history.py" --uav-id UAV-004 --task-id <task_id>
```

- `<task_id>` 是该机动作序列的 task_id，即 `<task_key>-<uav_id>`。
- `read_uav_status.py` 的 `--kind`：execution（执行状态）、ack（是否接收）、observations（感知观测）、perception（感知状态）、behavior（目标行为）。航段、解锁类问题看 execution。
- `READ_OK` 只说明读取成功，状态在 `payload` 里。`READ_FAILED`、超时、解析失败，以及诊断脚本返回 `ERROR`，都表示“状态未确认”，不能解释为没有目标、未收到指令或任务失败。
- 不要用 `rostopic echo` 的输出直接当 JSON 解析（它带 YAML 包装）。
- 解锁失败要区分：`COMMAND_REJECTED`（飞控拒绝，常见原因是健康检查未过）、`SERVICE_CALL_FAILED`（服务异常）、`STATE_CONFIRM_TIMEOUT`（请求被接受但未观察到解锁）。健康检查未过时报告实际原因，不反复解锁，也不重启控制器绕过。
- `SEGMENT_STALLED`（连续一段时间没有位移）和 `SEGMENT_TIMEOUT`（航段超时）：根据 execution 中 `timeout_evidence` 的目标、实际位置、剩余距离和规划状态判断原因，不凭距离和标称速度断定“时限太短”。
- 执行器会话（`executor_session_id`）变化只说明控制器重启过，不代表代码版本不同；重启后旧的暂停进度失效。
