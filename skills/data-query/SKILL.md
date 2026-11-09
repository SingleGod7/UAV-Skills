---
name: data-query
description: 用只读 SQL 查询资源库、状态库和地点库，保持记录字段原义；需要分配候选机时查询合格机及可用预算。用于资源和地点事实查询、相对位置基准查询及候选准备。
---

# 数据查询

## 可用函数

| 函数 | 脚本 | 表 |
|---|---|---|
| 资源库查询 | `query_uav_resource.py` | `uav_resource` |
| 状态库查询 | `query_uav_status.py` | `uav_status` |
| 地点库查询 | `query_location.py` | `geo_library` |

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/data-query/scripts/query_location.py" \
  --sql "SELECT * FROM geo_library WHERE region_name = '教学楼' OR alias LIKE '%教学楼%' LIMIT 3"
```

- `--sql` 只能是一条 `SELECT` 或 `WITH … SELECT`，外层双引号、内层单引号；每个脚本只能查自己的表。
- 按需要选字段，不必 `SELECT *`（地点库除外，查地点时可取全部字段）。
- 互不依赖的查询放在同一轮并行发出。

## 表字段

**uav_resource**（静态性能）

| 字段 | 说明 |
|---|---|
| `uav_id`、`alias` | 编号、别名 |
| `uav_model`、`uav_series` | 型号、系列 |
| `max_endurance`、`hover_endurance` | 最大续航、悬停续航，单位 min |
| `max_range`、`max_flight_radius` | 最大航程、最大飞行半径，单位 m |
| `cruise_speed`、`max_speed`、`max_up_speed`、`max_down_speed` | 速度，单位 m/s |
| `max_altitude` | 最大飞行高度，单位 m |
| `max_wind_speed`、`emi_resistance_level` | 抗风、抗电磁干扰等级 |
| `takeoff_mode`、`hover_accuracy`、`max_angular_velocity` | 起飞模式、悬停精度、最大角速度 |

**uav_status**（状态记录）

| 字段 | 说明 |
|---|---|
| `uav_id` | 编号 |
| `mission_status` | 空闲 / 执行中 / 离线 / 状态未知（由同步服务实时更新） |
| `longitude`、`latitude`、`altitude` | 记录的起飞点位置，不随飞行更新 |
| `battery_level` | 电量记录，% |
| `health_score` | 健康度 |
| `payload_status` | 载荷：高清相机 / 红外相机 / 高清+红外 / 激光雷达 |
| `flight_remain` | 剩余可飞行距离，m |
| `max_range_full`、`max_range_empty`、`load_range` | 满载航程、空载航程、载荷航程，m |
| `fov`、`camera_recognition_range`、`infrared_detection_range`、`lidar_detection_range` | 视场角、各类探测距离 |

**geo_library**（地点）

| 字段 | 说明 |
|---|---|
| `region_name`、`alias` | 正式名称；别名以 `;` 分隔 |
| `longitude`、`latitude`、`altitude` | 地点入口坐标与海拔 |
| `region_shape`、`region_length_m`、`region_width_m`、`region_area_m2` | 登记的任务区域：形状、东西长、南北宽、面积；为空表示未登记区域 |
| `region_center_longitude`、`region_center_latitude` | 任务区域中心，与入口不同 |

“默认位置”是一个别名，查询写 `alias LIKE '%默认位置%'`。
仅方向+距离的指定单机请求，参照该机当前位置的语义规则见 mission-planning；本库只提供起飞点记录，使用前应取得用户对该记录基准的确认，不能直接用地点库默认位置替代。

## 候选机与可用预算

`scripts/prepare_candidates.py` 查询当前资源、状态和机队配置，输入可包含 `uav_ids`、`exclude_uavs`、`payload` 字符串数组，以及 `flight_alt`（默认1.2米）。使用 AGENTS.md 中的 JSON 调用与结果引用方式：

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/data-query/scripts/prepare_candidates.py" --input '{"uav_ids":["UAV-001","UAV-003"],"flight_alt":1.2}'
```

省略 `uav_ids` 表示查询全部合格机；候选数不表示实际出动数。保留原预筛条件：三处记录齐全、空闲、电量大于30%、健康度大于60、剩余航程有效、载荷符合明确要求、机型升限满足输入航高。点名候选未全部合格返回 `CANDIDATE_EXCLUDED`，无合格机返回 `NO_CANDIDATE`。

`payload` 数组各项必须同时满足，每项按状态记录字符串包含匹配，不表示多个名称任选其一。用户只要求红外能力时可用 `["红外"]` 匹配“红外相机”或“高清+红外”；不能把两种替代名称同时放入数组，变成额外的双重要求。具体载荷不明时先查询原记录，不靠探测距离推断。

完整结果 `candidates` 包含机号、起飞原点、原始电量/航程/升限记录及 `range_budget`（米）、`time_budget`（秒）；`excluded` 列排除原因。预算保留20%电量，由工具计算，不把 `flight_remain` 或分钟制 `max_endurance` 直接改名作预算。本入口不查询任务地点、不计算路径、不决定归属。

## SQL 返回与异常

成功返回 `{"status":"OK","count":…,"truncated":…,"data":[…]}`，只读取 `data` 里实际返回的字段。

| 状态 | 处理 |
|---|---|
| `NOT_FOUND` | 没有记录：如实告知用户或询问，不猜测 |
| `INVALID_ARGUMENT` | 对照上面的表名和字段修正 SQL，最多重试一次 |
| `ERROR` | 报告数据库错误，停止依赖它的步骤 |
| `truncated=true` | 结果超过 100 行：收窄条件或分页，不把第一批当作全部 |

## 注意事项

- 地点匹配到多条时请用户确认是哪一个；查无结果时直接询问，不编造坐标。
- 不做表结构枚举（PRAGMA、sqlite_master），可用字段以本文件为准；禁止写入。
- 状态与资源通过 `uav_id` 关联；查电量、任务状态这类动态信息时每次重新查询，不复用对话里的旧结果。
- 状态库的位置是起飞点记录，不是飞行中的实时位置；运行中的状态以中央任务 `get` 为准。
- 载荷类型和探测距离是两种约束，不能互相推导。
