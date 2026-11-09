---
name: trajectory-generation
description: 基于静态避障地图查询单项航迹和地图，计算任务分配所需的转场与作业成本，并按已有求解归属和顺序生成每机具体航段、区域分工及航程估计。
---

# 航迹查询

单项查询沿用下面的 `semantic_map.py`。任务规划使用 `planning_routes.py` 的 `costs` 和 `build` 两种操作：前者计算数值成本，后者按已有分配生成最终航迹；两者均不调用分配算法。

## 任务规划的成本与航段

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/trajectory-generation/scripts/planning_routes.py" --input '<JSON对象>'
```

也可用 `--input-file <工作区内绝对路径>`，与 `--input` 互斥。成功返回 `artifact_file`、`data_fields` 和 `summary`，完整结果在该文件的 `data` 中。任何输入字段可用 `{"ref":"实际返回的artifact_file","select":"/data/字段"}` 引用；大矩阵和完整分配记录直接引用。

`operation=costs` 接收候选记录和已解析区域：

```json
{
  "operation": "costs",
  "candidates": {"ref": "候选查询实际返回的文件", "select": "/data/candidates"},
  "jobs": {"ref": "区域解析实际返回的文件", "select": "/data/jobs"},
  "task_type": "search"
}
```

- `task_type` 为 `search`（含无目标探索）或 `patrol`；`flight_alt` 默认1.2 m，是错高基准；`laps` 默认1且只用于巡逻；`speed` 可选，省略时转场1.2 m/s、搜索覆盖估计0.6 m/s。
- `laps` 是同一次任务内连续巡逻的圈数，不含按时启动、间隔等待、分趟返航或补能；周期任务不能通过换算总趟数填入 `laps`，先按 mission-planning 说明能力缺口并确认单趟备选。
- candidates 包含 `uav_id,origin,range_budget,time_budget,battery,flight_remain,max_altitude`；jobs 包含 `job_id,name,center,shape` 和形状尺寸，可带 `entrance,fixed_uav,area_source`。地点及机队数据由调用方提供，本入口不查询数据库。
- 返回 `/data/candidates`、`/data/jobs`（求解用编号与名称）、`/data/costs`、`/data/constraints`，可原样用于分配求解。约束保留区域的 `fixed_uav`；圆形巡逻要求单机负责，以保留完整圆周。用户另有数量或候选限制时须一并表达。
- `/data/prepared` 保存完整区域、候选和实际参数；`prepared_digest`用于发现与成本记录不一致的修改。覆盖轨迹只是航程估计，不代表保证覆盖完成。

`operation=build` 接收完整成本记录和完整分配结果：

```json
{
  "operation": "build",
  "cost_estimate": {"ref": "成本实际返回的文件", "select": "/data"},
  "allocation": {"ref": "分配实际返回的文件", "select": "/data"},
  "search_timeout": 240
}
```

- `search_timeout` 默认1200秒；航高、圈数、速度和区域沿用成本记录，不能在此覆盖。改这些输入后需要相应的新成本及分配证据。
- 校验分配的真实求解输入、结果、索引、预算、成本、约束和访问顺序；不手改归属。按参与机编号依次在基准航高上增加0.3 m，并检查地图与每架机上限；严格固定同高的多机请求不能用该错高策略代替。
- 返回 `task_key,jobs,assignments,unused,solver`。每项 assignment 含 `uav_id,origin,flight_alt,jobs,steps,estimated_range_m,estimated_battery_after`。steps 为 `transit`、`search` 或 `patrol`，兼容 flight-control 的动作组装输入；搜索步含 `box_min,box_max,timeout_sec`，巡逻步含 `waypoints,laps`。
- 最终分区航程须满足输入航程预算且剩余电量大于20%。该估计只覆盖水平转场和覆盖/巡逻长度；不包含起降、搜索超时上界或跟踪占用，`whole_mission_budget_complete=false`，不能称全任务续航已验证。

`ALLOCATION_EVIDENCE_MISMATCH` 表示分配记录与成本或原始求解不对应；`ALTITUDE_INFEASIBLE` 表示错高超过限制；`BUDGET_EXCEEDED` 表示最终几何超过预算。失败时报告具体约束，不修改归属或放宽参数求成功。

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/trajectory-generation/scripts/semantic_map.py" <子命令> …
```

## query-path：两点航线

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/trajectory-generation/scripts/semantic_map.py" query-path \
  --start-lon <起点经度> --start-lat <起点纬度> --start-alt 0 \
  --end-lon <终点经度> --end-lat <终点纬度> --end-alt 0 --flight-alt 1.2
```

| 参数 | 必填 | 说明 |
|---|:---:|---|
| `--start-lon/--start-lat/--start-alt` | 单起点时是 | 起点经纬度和地面海拔 |
| `--starts-json` | 多起点时是 | `[{"uav_id":"UAV-001","longitude":…,"latitude":…,"altitude":0},…]`，与单起点参数不能同时用 |
| `--end-lon/--end-lat/--end-alt` | 是 | 终点经纬度和地面海拔 |
| `--flight-alt` | 是 | 航高，(0, 3.0] m |

返回 `go_waypoints`、`back_waypoints`、`one_way_distance`、`total_distance`（m）；多起点时在 `results` 中按输入顺序逐项返回。

## query-area：区域航迹

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/trajectory-generation/scripts/semantic_map.py" query-area \
  --center-lon <中心经度> --center-lat <中心纬度> --center-alt 0 \
  --shape rectangle --length <东西长> --width <南北宽> --mode coverage --flight-alt 1.2 --lane 2
```

| 参数 | 必填 | 说明 |
|---|:---:|---|
| `--center-lon/--center-lat/--center-alt` | 是 | 区域中心（地点库的 `region_center_*`，不是入口） |
| `--shape` | 是 | `rectangle` 或 `circle` |
| `--length`、`--width` / `--radius` | 按形状必填 | 矩形东西长、南北宽 / 圆形半径，m |
| `--mode` | 是 | `boundary`：边界巡逻闭合环；`coverage`：覆盖航迹（搜索航程估算） |
| `--flight-alt` | 是 | 航高，(0, 3.0] m |
| `--lane` | 否 | 采样间距，默认 50；本场景建筑约 10 m，填 2 |
| `--start-lon/--start-lat` | 否 | 区域内的进入点，须成对给出 |

返回 `waypoints` 和 `path_length`（m）。`boundary` 另返回 `approach_waypoints`、`approach_length` 以及 `closed=true`、`boundary_complete=true`。

## query-map：地图概况

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/trajectory-generation/scripts/semantic_map.py" query-map
```

返回场景范围、航高范围、五栋楼的名称、入口与范围、起降区等；不包含人物或物品的位置。

## 异常

| 状态或信息 | 含义与处理 |
|---|---|
| `INVALID_ARGUMENT` | 检查经纬度、航高、形状与尺寸、模式等参数，修正后最多重试一次 |
| `SEMANTIC_MAP_OUTSIDE_CAMPUS`、`SEMANTIC_MAP_AREA_OUTSIDE_MAP` | 超出场景地图：报告不可执行 |
| `GLOBAL_ROUTE_GOAL_OCCUPIED`、`GLOBAL_ROUTE_UNREACHABLE` | 目标点被障碍占据或不可达：报告，不改用直线 |
| `SEMANTIC_MAP_BOUNDARY_BLOCKED` | 区域边界整圈不可通行（建筑边界会穿墙）：不能巡逻该区域 |
| `ERROR` | 报告错误并停止 |

## 注意事项

- 经纬度为 WGS84；高度、尺寸、距离单位为米。返回的航点和距离直接使用，不修改。
- `coverage` 的航迹和长度只用于估算航程，实际搜索由 FUEL 自主探索完成。
- 场景航高上限 3.0 m。
- 规划报告中的地图类型取实际 `map_context`；`semantic/mock=false` 与 `mock/mock=true` 分别表述，不因数据是静态快照就把它称为mock。
