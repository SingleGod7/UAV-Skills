---
name: spatial-compute
description: 地理空间换算：计算相对位置、将已查询的地点记录与用户范围整理为作业区域，以及生成圆形或矩形区域的FUEL搜索框。
---

# 空间计算

## 可用函数

| 函数 | 用途 |
|---|---|
| `calc_offset.py` | 参考点 + 方位角 + 距离 → 目标点经纬度 |
| `build_fuel_box.py` | 区域中心 + 形状尺寸 + 航高 → FUEL 搜索框 `box_min`、`box_max` |
| `resolve_jobs.py` | 已查询地点记录 + 用户明确范围 → 数值作业区域 |

空间单项可独立计算。完整任务的最终航迹包含按实际分区生成的搜索框；数值分配本身不生成搜索框。

## resolve_jobs.py

按 AGENTS.md 的 JSON 调用方式，输入 `{jobs:[...],places:[...]}`。`places` 是地点查询返回的真实记录数组；全部作业已给中心坐标时可省略。脚本不查库、不猜地名或范围。

每个作业使用原格式：`{place:"教学楼"}`，或 `{center_lon:...,center_lat:...,shape:"rectangle",length:...,width:...}`。可加 `job_id`、`name`；用户指定某机负责时加 `uav_id`。地名作业的用户形状、尺寸、中心覆盖相应登记字段；圆形必须给 `radius`。只有登记点且用户未给范围时返回缺参数，不能把偏移距离当半径。

返回 `jobs`，含 `job_id/name/center/entrance/shape/尺寸/fixed_uav/area_source`。区域中心与入口分别保留；有登记区域时取其中心，用户为登记点明确给出范围时以该点为中心。未找到、同名多记录、缺尺寸或不完整坐标分别返回 `NOT_FOUND`、`AMBIGUOUS`、`MISSING_PARAMETER`，非法数值返回 `INVALID_ARGUMENT`。这些结果只说明所提供记录与参数能否构成区域，不认证用户意图。

## calc_offset.py

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/spatial-compute/scripts/calc_offset.py" \
  --lon <参考点经度> --lat <参考点纬度> --bearing <方位角> --distance <距离>
```

| 参数 | 必填 | 说明 |
|---|:---:|---|
| `--lon`、`--lat` | 是 | 参考点经纬度（WGS84） |
| `--alt` | 否 | 参考点海拔，默认 0 m |
| `--bearing` | 是 | 方位角，正北为 0°，顺时针：北 0、东北 45、东 90、东南 135、南 180、西南 225、西 270、西北 315 |
| `--distance` | 是 | 参考点到目标点的距离，m，须 > 0 |

成功返回 `center_lon`、`center_lat`（目标点）以及回显的输入。参考点先按 mission-planning 的“相对基准”确认，再由 data-query 提供：地点有登记区域时用区域中心 `region_center_longitude/region_center_latitude`，否则用地点坐标；指定单机时不能擅自换成地点库默认位置，也不能把起飞点记录冒充实时位置。`--alt` 不用传。

## build_fuel_box.py

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/spatial-compute/scripts/build_fuel_box.py" \
  --center-lon <中心经度> --center-lat <中心纬度> --shape rectangle --length <东西长> --width <南北宽> --flight-alt <航高>
```

| 参数 | 必填 | 说明 |
|---|:---:|---|
| `--center-lon`、`--center-lat` | 是 | 区域中心 |
| `--shape` | 是 | `circle` 或 `rectangle` |
| `--radius` | 圆形必填 | 半径，m |
| `--length`、`--width` | 矩形必填 | 东西长、南北宽，m |
| `--flight-alt` | 是 | 任务航高，m，须 > 0.3 |

返回 `box_min=[西经度,南纬度,0.3]`、`box_max=[东经度,北纬度,航高+0.5]`。圆形区域返回外接正方形。

## 异常

| 状态 | 处理 |
|---|---|
| `MISSING_PARAMETER` | 上下文里有对应信息就补上；没有就停止并询问用户 |
| `INVALID_ARGUMENT` | 检查经纬度范围、正数约束和形状对应的尺寸参数，修正后最多重试一次 |
| `ERROR` | 停止，报告错误，不自行计算坐标或边界 |

## 注意事项

- 坐标为 WGS84 经纬度；距离、尺寸、高度单位为米。
- 参考点到目标点的距离不是区域尺寸，不能拿来当半径或长宽。
- 函数返回值直接使用，不重算、不取整。
