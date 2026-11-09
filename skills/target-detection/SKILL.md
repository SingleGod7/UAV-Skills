---
name: target-detection
description: 管理每架无人机的目标感知：把用户要找的目标整理成目标清单，启动、更新、查询和停止感知会话，查询多目标搜索进度，并提供中央任务注册所需的感知引用。用于带目标的搜索任务下发前的感知准备，以及“开启/停止识别”“还剩哪些目标没找到”这类单项请求。
---

# 目标感知

## 目标清单

用户要找的每个目标拆成一项。同一任务的所有参与机共用同一份清单和同一个 `search_id`：

```json
{"search_id":"<task_key>","targets":[
  {"target_id":"red_person","category":"person","description":"穿红色上衣、黑色裤子的人"},
  {"target_id":"pink_case","category":"suitcase","description":"浅粉色、贴有四张贴纸的行李箱"}]}
```

| 字段 | 规则 |
|---|---|
| `search_id` | 本任务唯一，用最终航迹结果的 `task_key`；目标清单改动时换新 ID |
| `target_id` | 清单内唯一，用英文小写和下划线概括目标，如 `red_person`、`black_backpack`；不超过 80 个字符 |
| `category` | 检测模型的英文粗类别（COCO 类名）：人→`person`，汽车→`car`，卡车→`truck`，自行车→`bicycle`，背包/双肩包→`backpack`，手提包→`handbag`，行李箱→`suitcase` |
| `description` | 目标的具体外观，可以用中文，供中央语义复核使用 |

- 同一类别但外观不同的目标，拆成不同的 `target_id`，不合并。
- `targets` 表达按特征匹配的对象，不是已去重的物理实例计数。多个 ID 使用同一类别和同一描述，不能证明找到了多个不同实体；不得用复制条目来表示“至少N辆/人/件才完成”。当前不支持这类去重数量完成条件，需明确说明，不能以目标配置 dry-run 通过替代它。不同外观目标的多目标搜索不受此限制。
- 颜色、衣着写在 `description`，不写进 `category`（不要写成 `red person`）。
- 中央任务 `authorize` 的 `targets` 使用同一份清单（只取 `targets` 数组）。
- 探索、建图任务没有目标，不启动感知。

## 命令

```bash
source "$ROS_SETUP" && source "$CATKIN_SETUP" && \
/usr/bin/python3 "$MIGRATION_ROOT/scripts/uav_perception.py" <命令> --uav-id <uav_id> --query-json '<JSON>'
```

| 命令 | 用途 | `--query-json` | 成功返回 |
|---|---|---|---|
| `status` | 查该机感知是否在运行 | 不需要 | 状态 JSON；`NOT_RUNNING` 表示未运行 |
| `start` | 启动感知并设置目标清单 | 目标清单对象 | `status=STARTED`、`ready=true`、`session_id`、`query_id` |
| `query` | 感知已在运行时，换成新的目标清单 | 目标清单对象 | `state=ACCEPTED`、`query_id` |
| `progress` | 查已找到和未找到的目标 | `{"search_id":"<search_id>"}` | 已找到与剩余的 `target_id` |
| `stop` | 停止该机感知 | 不需要 | `state=STOPPED` |

感知始终以默认的 `detect-and-continue`（只检测，不接管飞行）模式运行，不加 `--mode`。

**只规划时的校验**：不需要 ROS，命令为

```bash
python3 "$MIGRATION_ROOT/scripts/uav_perception.py" start --uav-id <uav_id> --query-json '<目标清单对象>' --dry-run
```

返回启动预览（含 `backend`、`mode`、`categories`），不启动感知；有这份预览即校验通过，失败时输出“感知操作失败: …”。

该预览只证明清单格式和机号配置合法，不证明目标实际存在、实例去重、数量达标或完成策略可实现。

## 下发前的感知准备

1. 对每架参与机（可并行）：先 `status`；返回 `NOT_RUNNING` 用 `start`，已在运行用 `query` 换成本任务清单（会话不变，`query_id` 换新）。
2. 全部成功后，用 `mission_client.py status` 读 `fleet.<uav_id>.perception`，作为该机 `register-plan` 的 `perception`；这里的值才是中央认可的当前会话。
3. 任一机失败：停止流程，不下发飞行；本任务新启动的感知全部 `stop`。

## 单项请求

- 开启识别：需要明确是哪架机、找什么目标；多架机在线而用户没说明时要问清。不生成飞行动作。
- “还剩哪些目标”：`progress` 查一次后回答，不循环查询。
- 停止识别：`stop`。

## 发现通知的回复

收到目标发现通知时，用一两句话说明哪架机发现了哪个目标；不输出事件 ID、坐标、日志或推理过程，也不承诺持续监控。感知不可用的通知同样用一句话说明。

## 失败处理

| 情况 | 处理 |
|---|---|
| 目标清单格式报错 | 按报错修正一次 |
| “感知未产生有效观测”、启动超时 | 报告该机感知启动失败，停止依赖它的下发 |
| “本机感知实例已经运行或正在启动” | 改用 `query` |
| “先停止本机目标行为，再修改其感知查询” | 该机有旧目标行为在运行：报告，不自行停止正在执行的任务 |
| “感知查询未确认”或被拒绝 | 报告；不能把旧会话当作新清单已生效 |

## 注意事项

- 跟踪由中央任务策略执行（见 flight-control 的 authorize）。感知只用默认的检测模式启动；不要用 `track-target`、`detect-and-return` 模式，也不要用 `start_target_tracking.py` 启动旧的目标行为节点。
- 目标是否找到以中央复核为准：逐个 `target_id` 看 MATCH。YOLO 框只是候选；NO_MATCH、UNKNOWN 都不等于目标不存在。
- 检测不到粗类别时不会触发语义复核，不能据此断言现场没有目标。
- 目标位置缺失时如实说明，不用无人机位置代替目标位置。
