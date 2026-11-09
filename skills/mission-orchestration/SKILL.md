---
name: mission-orchestration
description: 根据已有候选无人机预算、区域作业和数值成本，求解任务归属、共担与访问顺序。用于分配咨询或任务方案中的分配计算；allocate 只负责分配求解。
---

# 任务归属与访问顺序

入口为 `scripts/allocate.py`，产物类型为 `allocation`。已知的纯数值输入可以独立求解；地点、机队记录、地图成本和最终航迹是其他能力的输入或结果，不由本脚本查询或生成。

```bash
python3 "$PLAN_AGENT_WORKSPACE/skills/mission-orchestration/scripts/allocate.py" --input-file <输入JSON的绝对路径>
```

也可用 `--input '<JSON对象>'`，两种输入方式互斥。

## 输入

| 字段 | 契约 |
|---|---|
| `candidates` | 1..64 项；唯一 `uav_id`，正数 `range_budget`（m）与 `time_budget`（s）；其他机队元数据不参与求解 |
| `jobs` | 1..64 项；唯一 `job_id`，可带 `name` |
| `costs.transfer_range` | 每个候选机对应一张 `(N+1)×(N+1)` 转场距离矩阵，单位 m |
| `costs.transfer_time` | 同样索引的转场时间矩阵，单位 s |
| `costs.work_range` | 按 jobs 顺序给出的 N 项完整作业距离，非负数 |
| `costs.work_time` | 同序的 N 项完整作业时间，正数 |
| `costs.reachable` | 每个候选机对应可执行的 `job_id` 数组 |

矩阵索引 0 是该机基地，1..N 严格对应 jobs 顺序。对角线为 0 或 null，其他有效值须为非负有限数。不可达弧可为 null，但必须由 reachable 排除；两个可执行节点之间缺成本返回 `MISSING_COST`。作业向量也接受 N+1 项格式，首项为 0 或 null。

候选预算、作业和成本已有产物时，直接引用相应字段。例如已有成本产物的输入形式如下，文件路径使用本轮工具实际返回值：

```json
{
  "candidates": {"ref": "<成本产物绝对路径>", "select": "/data/candidates"},
  "jobs": {"ref": "<成本产物绝对路径>", "select": "/data/jobs"},
  "costs": {"ref": "<成本产物绝对路径>", "select": "/data/costs"},
  "constraints": {"ref": "<成本产物绝对路径>", "select": "/data/constraints"}
}
```

### 分配约束

可选 `constraints` 只表达归属和实际参与数量：

- `fixed_assignments:{job_id:uav_id}`：该作业由指定机独占。
- `allowed_assignments:{job_id:[uav_id]}`：该作业只能从这些机号中选择；未列作业沿用物理可达候选。允许集合与 `costs.reachable` 取交集，交集为空或与固定归属冲突时返回不可行。
- `single_owner_jobs:[job_id]`：该作业只能由一架机负责，机号由算法选择。
- `required_drones:N`：恰好 N 架实际作业，待命机不计数；候选不足或其他约束无法满足时返回不可行。

未限定独占的作业最多两机共担，以均分作业成本估计。候选名单不代表全部必须出动。`costs.reachable` 表达物理可达性，`allowed_assignments` 表达用户限定的逐作业候选，两者共同约束固定归属；保留完整原始成本供校验。逐作业允许集合与精确出动数分别表达，只有用户指定数量时才设置 `required_drones`。已有约束与用户新增约束同时有效，不能覆盖掉已有固定归属或单机要求。

可选 `solver_options`：`time_limit_sec` 默认为 20、上限 120；`weights` 支持 `makespan,total_range,total_wait,uav_count`，均非负且至少一项大于零。省略 weights 使用求解器默认值；显式只给部分权重时其余为零。

多个独立并行任务可用 `groups:[{group_id,jobs,costs,candidates?}]` 代替顶层 jobs/costs，全局 candidates 和 constraints 保留。各组 job_id 全局唯一，成本只对应该组的作业数量与顺序；每架机最多属于一个组。组内成本的机号须为全局候选子集；可选组内 candidates 用于核对同机预算。此形式只处理归属互斥，不合并任务目标或动作。

## 输出与解释

成功返回产物文件，完整 `data` 包含：

- `assignments:[{uav_id,job_ids,estimated_range_m}]`：实际执行机及其访问顺序。
- `owners:{job_id:[uav_id]}`、`unused`：作业归属与未参与候选。
- `index`：成本矩阵对应的原机号与作业编号顺序。
- `constraints`、`solver`、`solver_input`、`solver_result`：实际约束、求解状态、同次求解的数值输入与原始结果，供后续结果复核。

groups 形式另有 `allocations_by_group` 与 `job_groups`，可按组获取归属和顺序。组视图来自同一次全局联合求解，使用原始求解证据时按其 `global_uav_indices/global_job_indices` 映射。

`estimated_range_m` 是给定成本与共担假设下的条件估计；`estimated_only=true`，不能当成最终航迹或完整任务续航已经通过。原始求解器的等待变量可能含松弛量，`endurance_used/finish_time` 不作为实际耗时估计。数值结果校验不认证输入是否符合用户意图；具体避障航点、区域切分、航高、搜索框与剩余电量不在本结果中。

分配咨询交付实际执行机、作业顺序、未用候选和上述估计范围；`solver.optimal=false` 时说明“可行但未证明最优”。

## 失败

`MISSING_PARAMETER` 表示缺输入；`INVALID_ARGUMENT` 表示编号、数值、索引或字段不合法；`MISSING_COST` 表示缺少所需成本；`BUDGET_MISMATCH` 表示组内外同机预算不一致；`NO_FEASIBLE_ALLOCATION` 表示给定候选、成本与约束无可行分配。`SOLVER_TIMEOUT/SOLVER_FAILED/SOLVER_UNAVAILABLE` 表示求解运行问题。报告实际状态和缺项，不把失败结果作为有效分配。
